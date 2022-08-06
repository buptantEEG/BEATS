/*
compile:
gcc -o rpi_ads1299_v3 rpi_ads1299_v3.c rpi_dma_utils.c -Wall -lwiringPi
*/


#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include "rpi_dma_utils.h"

#include <wiringPi.h>


// -------- RPI SPI -------- //
// SPI clock frequency
#define MIN_SPI_FREQ    10000
#define MAX_SPI_FREQ    20000000
#define SPI_FREQ        4000000

#define SPI0_CE_NUM     0
// SPI 0 pin definitions
#define SPI0_CE0_PIN    8
#define SPI0_CE1_PIN    7
#define SPI0_MISO_PIN   9
#define SPI0_MOSI_PIN   10
#define SPI0_SCLK_PIN   11

// SPI registers and constants
#define SPI0_BASE       (PHYS_REG_BASE + 0x204000)
#define SPI_CS          0x00
#define SPI_FIFO        0x04
#define SPI_CLK         0x08
#define SPI_DLEN        0x0c
#define SPI_DC          0x14
#define SPI_FIFO_CLR    (3 << 4)
#define SPI_RX_FIFO_CLR (2 << 4)
#define SPI_TX_FIFO_CLR (1 << 4)
#define SPI_TFR_ACT     (1 << 7)
#define SPI_DMA_EN      (1 << 8)
#define SPI_AUTO_CS     (1 << 11)
#define SPI_RXD         (1 << 17)
#define SPI_CE0         0
#define SPI_CE1         1
#define SPI_CSVAL       (1 << 2)       // Additional CS register settings, SPI mode 0b01

// SPI register strings
char *spi_regstrs[] = {"CS", "FIFO", "CLK", "DLEN", "LTOH", "DC", ""};

int init_spi(int hz);
void spi_clear(void);
void spi_cs(int set);
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len);
void spi_disable(void);
void disp_spi(void);

// -------- --------//

// Non-cached memory size
// #define MAX_SAMPS       40//00//*10 // Currently used for the number of samples you want to receive, adjustable

// #define BUFF_LEN        (MAX_SAMPS * SAMP_SIZE)
// #define MAX_BUFFS       2
// #define VC_MEM_SIZE     (PAGE_SIZE + (BUFF_LEN * MAX_BUFFS))

// Virtual memory pointers to acceess peripherals & memory
extern MEM_MAP gpio_regs, dma_regs, clk_regs, pwm_regs;
MEM_MAP vc_mem, spi_regs, usec_regs;

void terminate(int sig);
void map_devices(void);

// -------- FIFO -------- //
// File descriptor for FIFO
int fifo_fd = 0;
uint32_t fifo_size;

int create_fifo(char *fname);
int open_fifo_write(char *fname);
int write_fifo(int fd, void *data, int dlen);
uint32_t fifo_freespace(int fd);
void destroy_fifo(char *fname, int fd);

// fcntl constant to get free FIFO length
#define F_GETPIPE_SZ 1032
// -------- -------- //

// -------- ADS1299 -------- //
// ADS1299 COMMAND
#define CMD_RESET 0x06
#define CMD_START 0x08
#define CMD_STOP 0x0A
#define CMD_RDATAC 0x10
#define CMD_SDATAC 0x11
#define CMD_RDATA 0x12
#define CMD_RREG 0x20
#define CMD_WREG 0x40

#define PGA 24
#define VREF 4.5
#define ADC_VOLTAGE(n)  (((n) * VREF) / 0x7FFFFF / PGA)

void send_command(uint8_t command);
void write_reg(uint8_t address, uint8_t value);
void read_reg(uint8_t address);
void read_reg_all();
void setup();
void initialize();
void change_sample_rate(int sample_rate);
void change_mux(int mux);

// void test_signal();

#define MAX_CHIPS 3
// -------- -------- //


// --------  -------- // 
// Form ping-pong buffer with two arrays
#define MAX_BUFF_SAMPLES 40  // Maximum number of samples stored in each buff
#define SAMPLE_SIZE       27 // single chip, bytes per sample
uint8_t rx_buff0[MAX_BUFF_SAMPLES*(1+MAX_CHIPS*SAMPLE_SIZE)];
//Store every 8 bit data of the received SPI, 1 refers to the useless 8bit returned when sending the RDATA command
uint8_t rx_buff1[MAX_BUFF_SAMPLES*(1+MAX_CHIPS*SAMPLE_SIZE)];
int which_rx_buff; // 0 and 1, indicating buff0 and buff1. -1 means the buff is not available
int rx_buff0_status = 0; // Status, 1 means full and can be read
int rx_buff1_status = 0; 
int buff_samples = 0; // The number of samples collected by the current buff
#define VOLTAGE_BUFFLEN  5000+ MAX_CHIPS*5000
char voltage_buff[VOLTAGE_BUFFLEN]; // Store the converted decimal value for output

int stop_receive_flag = 0; // flag to end interrupt detection
int end_program_flag = 0; // flag to end the entire program


#define VC_MEM_SIZE     (PAGE_SIZE + (MAX_BUFF_SAMPLES * SAMPLE_SIZE))

// -------- DMA -------- //
void dma_receive_data(); // Use DMA to send SPI commands to receive data

void dma_wait(int chan); // Wait until DMA is complete
void ads1299_tx_data(void *buff, int chip_num);
// -------- -------- //

void ISR_function();
void disp_array(uint8_t *array);
int format_data(uint8_t *rx_buff); // Combine the received 8bit data into decimal form
void streaming_output(); // Continuously output to FIFO
int manage_output_buff();


// ------ Timestamp and Timing -------- //
# include <sys/time.h>
struct timeval start, end;
/*
    struct timeval {
　　        long tv_sec; // seconds
　　        long tv_usec; // microseconds
    }
*/

// -------- command line --------//
#define NORMAL_ELEC_INPUT 0
#define INPUT_SHORTED 1
#define TEST_SIGNAL 5

// Command-line variables
int mux = TEST_SIGNAL, sample_rate = 4000, chip_num = 1, detect_enable = 0;
uint32_t total_samples, target_samples;
char *fifo_name;


int main(int argc, char *argv[])
{
   int args = 0;
    printf("argc:%d\n", argc);
    while (argc > ++args)               // Process command-line args
    {
        if (argv[args][0] == '-')
        {
            switch (toupper(argv[args][1]))
            {
            case 'N':                   // -N: number of total samples
                if (args >= argc-1 || !isdigit((int)argv[args+1][0]) ||
                    (target_samples = atoi(argv[++args])) < 1)
                    fprintf(stderr, "Error: no target samples\n");
                break;
            case 'P':                   // -P:  pipe name, stream into named pipe (FIFO)
                if (args >= argc-1 || !argv[args+1][0])
                    fprintf(stderr, "Error: no FIFO name\n");
                else
                    fifo_name = argv[++args];
                break;
            case 'R':                   // -R: sample rate (samples/sec)
                if (args >= argc-1 || !isdigit((int)argv[args+1][0]))
                    fprintf(stderr, "Error: no sample rate\n");
                else
                {
                    int input_value = atoi(argv[++args]);
                    if(input_value==250 || input_value==500 || input_value==1000 || input_value== 2000 || input_value== 4000)
                        sample_rate = input_value;
                    else
                        fprintf(stderr, "Error: invalid sample rate\n");
                }
                printf("sample_rate:%d\n", sample_rate);
                break;
            case 'C': // -C:number of chips
                if (args >= argc-1 || !isdigit((int)argv[args+1][0]))
                    fprintf(stderr, "Error: no number of chips\n");
                else
                {
                    int input_value = atoi(argv[++args]);
                    if(input_value>0 && input_value<=MAX_CHIPS)
                        chip_num = input_value;
                    else
                        fprintf(stderr, "Error: invalid number of chips\n");
                }
                printf("number of chips:%d\n", chip_num);
                break;
            case 'M':                 // -M:Input Multiplexer
                if (args >= argc-1 || !isdigit((int)argv[args+1][0]))
                    fprintf(stderr, "Error: no Input Multiplexer\n");
                else
                {
                    int input_value = atoi(argv[++args]);
                    if(input_value==NORMAL_ELEC_INPUT || input_value==INPUT_SHORTED || input_value==TEST_SIGNAL)
                        mux=input_value;
                    else
                        fprintf(stderr, "Error: invalid Input Multiplexer\n");
                }
                printf("Input Multiplexer:%d\n", mux);
                break;
            case 'D':  // -D:Detect enable
                detect_enable = 1;
                printf("Lead Off Detection enable. \n");
                break;
            case 'H': // -H:Help
                printf("Here is the manual. \n");
                printf("-N: number of total samples. \n");
                printf("-P:  pipe name, stream into named pipe (FIFO) \n");
                printf("-R: sample rate (samples/sec) \n");
                printf("-C:number of chips \n");
                printf("-M:Input Multiplexer. options: 0 for nornam input elec,1 for input shorted,5 for test signal.\n");
                printf("-D:Detect enable. \n");
                printf("-H:Help. \n");
                break;
            default:
                printf("Error: unrecognised option '%s'\n", argv[args]);
                exit(1);
            }
        }
    }

    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);

    init_spi(SPI_FREQ);
    wiringPiSetup();

    setup();
    initialize();
    change_sample_rate(sample_rate);
    change_mux(mux);

    // test_signal();
    read_reg_all();

    if(target_samples && fifo_name)
    {
        printf("target_samples:%d\n", target_samples);

        if (create_fifo(fifo_name))
        {
            create_fifo(fifo_name); 
            printf("Created FIFO '%s'\n", fifo_name);

            send_command(CMD_START);

            while(1)
            {
                streaming_output();

                if (end_program_flag==1)
                {
                    printf("total_samples:%d, sample_rate:%d \n", total_samples, sample_rate);
                    long timeuse =1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec;
                    printf("time=%f\n",timeuse /1000000.0);
                    break;
                }
            }
        }

       
    }
    else if (fifo_name)
    {
        if (create_fifo(fifo_name))
        {
            create_fifo(fifo_name); 
            printf("Created FIFO '%s'\n", fifo_name);

            send_command(CMD_START);

            while(1)
            {
                streaming_output();

                if (end_program_flag==1)
                {
                    printf("total_samples:%d, sample_rate:%d \n", total_samples, sample_rate);
                    break;
                }
            }
        }
    }

    terminate(0); 
}

void ISR_function()
{
    if(target_samples){
        if(total_samples == 0) gettimeofday(&start, NULL );;
        if(total_samples == target_samples){
            gettimeofday(&end, NULL );
            if(stop_receive_flag != -1)
                stop_receive_flag = 1;

            // total_samples = total_samples+1; 
        }
    }


    if(stop_receive_flag==0)
    {
        dma_receive_data();
    }
    else if(stop_receive_flag==1) 
    {
        dma_wait(DMA_CHAN_A);
        printf("step1\n");
        // send_command(CMD_STOP);  
        printf("step2\n");
        system("gpio unexport 27"); 
        stop_receive_flag = -1; 
        end_program_flag = 1; 
    }
    
}

// Continuous output to FIFO
void streaming_output()
{
    int write_len;

    // write data
    if (!fifo_fd)
    {
        printf("fifo check point1 \n");
        fifo_fd = open_fifo_write(fifo_name);
        printf("fifo_fd:%d \n", fifo_fd);
        if (fifo_fd > 0)
        {
            wiringPiISR(2, INT_EDGE_FALLING, &ISR_function); 
            
            printf("Started streaming to FIFO '%s'\n", fifo_name);
            fifo_size = fifo_freespace(fifo_fd);
            printf("fifo_size: %d \n", fifo_size);
        }
    }
    if (fifo_fd)
    {
        // printf("fifo check point2 \n");
        write_len = manage_output_buff();
        if(write_len > 0)
        {
            if (!write_fifo(fifo_fd, voltage_buff, write_len))
            {
                printf("Stopped streaming\n");
                if(stop_receive_flag!=-1)
                    stop_receive_flag = 1;
                close(fifo_fd);       
                fifo_fd = 0;
                usleep(100000);
            }
        }
    }
    // printf("fifo check point3 \n");

}

void disp_array(uint8_t *array)
{
    for (int i=0; i<MAX_BUFF_SAMPLES*28; i++)
    {
        printf("%02X ", array[i]);
        if((i+1)%28==0)
            printf("\n");
    }

}

int manage_output_buff()
{
    int len=0;
    if(rx_buff0_status==1)
    {
        if(rx_buff1_status==1) // two buffers are full
        {
            rx_buff0_status=rx_buff1_status=0;
            printf("overrun\n");
            return 0;
        }
        // formatting data
        len = format_data(rx_buff0);
        rx_buff0_status = 0;
        // ( XXX ? format_data(rx_buff0) : format_data(rx_buff1) );
    }
    else if(rx_buff1_status==1)
    {
        if(rx_buff0_status==1) // two buffers are full
        {
            rx_buff0_status=rx_buff1_status=0;
            printf("overrun\n");
            return 0;
        }
        // formatting data
        len = format_data(rx_buff1);
        rx_buff1_status = 0;
    }
    return len;
}

int format_data(uint8_t *rx_buff)
{
    // printf("format_data check point1 \n");
    int slen = 0;
    uint32_t c_raw; // Raw data in complement form
    int32_t raw; // signed raw data
    for (int i_samples=0; i_samples<MAX_BUFF_SAMPLES; i_samples++)
    {
        int sample_start_point = (1+27*chip_num)*i_samples;
        for(int j_channels=0; j_channels<9*chip_num; j_channels++)
        {
            int channel_start_point = sample_start_point + 3*j_channels + 1; 
            // concatenate
            c_raw = ((uint32_t)rx_buff[channel_start_point]<<16) + ((uint32_t)rx_buff[channel_start_point+1]<<8) + (uint32_t)rx_buff[channel_start_point+2]; //24bit拼接
            if(j_channels==0)
            {
                // slen += sprintf(&voltage_buff[slen], "%06X" , c_raw ); // save as 0x
                slen += sprintf(&voltage_buff[slen], "%d", c_raw); // save as decimal
            }
            else if(j_channels%9==0) 
            {
                // slen += sprintf(&voltage_buff[slen], ",%06X" , c_raw );
                slen += sprintf(&voltage_buff[slen], ",%d", c_raw); // save as decimal
            }
            else
            {
                raw = (c_raw>0x7FFFFF) ? (-1)*(((~c_raw)&0x007FFFFF)+1) : c_raw;
                slen += sprintf(&voltage_buff[slen], ",%1.9f" , ADC_VOLTAGE(raw) );
            } 
            // debug
            // printf("%02X %02X %02X:%X,", rx_buff[channel_start_point], rx_buff[channel_start_point+1], rx_buff[channel_start_point+2], c_raw);
            // printf("%02X%02X%02X,", rx_buff[channel_start_point], rx_buff[channel_start_point+1], rx_buff[channel_start_point+2]);

            // printf("%X:%d -> %X:%d:%1.9f, ", c_raw, c_raw, raw, raw, ADC_VOLTAGE(raw));

        }
        slen += sprintf(&voltage_buff[slen], "\n");
        //debug
        // printf("\n");
    }
    // printf("slen:%d\n", slen);
    // printf("format_data check point2 \n");

    return(slen);
}

// ——————————————————DMA———————————————— //
void dma_receive_data()
{
    MEM_MAP *mp = &vc_mem;
    DMA_CB *cbs=mp->virt;
    uint32_t dlen, *txdata=(uint32_t *)(cbs+2);
    uint8_t *rxdata=(uint8_t *)(txdata+0x40);

    enable_dma(DMA_CHAN_A);
    enable_dma(DMA_CHAN_B);
    dlen = 1+27*chip_num;
    txdata[0] = (dlen << 16) | SPI_TFR_ACT | SPI_CSVAL;
    ads1299_tx_data(&txdata[1], chip_num);

    cbs[0].ti = DMA_SRCE_DREQ | (DMA_SPI_RX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_DEST_INC;
    cbs[0].tfr_len = dlen;
    cbs[0].srce_ad = REG_BUS_ADDR(spi_regs, SPI_FIFO);
    cbs[0].dest_ad = MEM_BUS_ADDR(mp, rxdata);
    
    cbs[1].ti = DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC;
    cbs[1].tfr_len = dlen + 4;
    cbs[1].srce_ad = MEM_BUS_ADDR(mp, txdata);
    cbs[1].dest_ad = REG_BUS_ADDR(spi_regs, SPI_FIFO);

    *REG32(spi_regs, SPI_DC) = (8 << 24) | (4 << 16) | (8 << 8) | 4;
    *REG32(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_DMA_EN | SPI_AUTO_CS | SPI_FIFO_CLR | SPI_CSVAL;
    *REG32(spi_regs, SPI_DLEN) = 0;

    // if(total_samples == 0) start = clock();

    start_dma(mp, DMA_CHAN_A, &cbs[0], 0);
    start_dma(mp, DMA_CHAN_B, &cbs[1], 0);

    // if(total_samples == target_samples) finish = clock();

    
    if(which_rx_buff==0)
    {
        memcpy((rx_buff0 + dlen*buff_samples), rxdata, dlen);
        buff_samples++;
        if(buff_samples==MAX_BUFF_SAMPLES)
        {
            which_rx_buff=1;
            rx_buff0_status = 1;
            buff_samples=0;
        }
    }
    else if(which_rx_buff==1)
    {
        memcpy((rx_buff1+ dlen*buff_samples), rxdata, dlen);
        buff_samples++;
        if(buff_samples==MAX_BUFF_SAMPLES)
        {
            which_rx_buff=0;
            rx_buff1_status = 1;
            buff_samples=0;
        }
    }


    total_samples++;
    
}


// Return Tx data for ADS1299
void ads1299_tx_data(void *buff, int chip_num)
{
    uint8_t txd[1+chip_num*27]; // = {0};
    memset(txd, 0, sizeof(txd));
    txd[0] = CMD_RDATA;
    memcpy(buff, txd, sizeof(txd));

    // if(chip_num==1)
    // {
    //     uint8_t txd[28]={CMD_RDATA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //     memcpy(buff, txd, sizeof(txd));
    // }
    // else if(chip_num==2)
    // {
    //     uint8_t txd[1+2*27]={CMD_RDATA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    //                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //     memcpy(buff, txd, sizeof(txd));
    // }

    // return(sizeof(txd));
}

// Wait until DMA is complete
void dma_wait(int chan)
{
    int n = 1000;

    do {
        usleep(100);
    } while (dma_transfer_len(chan) && --n);
    if (n == 0)
        printf("DMA transfer timeout\n");
}

// -------- ADS1299 -------- //
void send_command(uint8_t command)
{
    uint8_t txd[1] = {command};
    uint8_t rxd[1];
    spi_cs(1);
    spi_xfer(txd, rxd, sizeof(txd));
    usleep(1);
    spi_cs(0);
}

void write_reg(uint8_t address, uint8_t value)
{
    uint8_t txd[3] = {CMD_WREG|address, 0x00, value};
    uint8_t rxd[3];
    spi_cs(1);
    spi_xfer(txd, rxd, sizeof(txd));
    usleep(1);
    spi_cs(0);
}

void read_reg(uint8_t address)
{
    uint8_t txd[3] = {CMD_RREG|address, 0x00, 0x00};
    uint8_t rxd[3];
    spi_cs(1);
    spi_xfer(txd, rxd, sizeof(txd));
    spi_cs(0);
    printf("%#x :%#x %#x %#x\n",address, rxd[0],rxd[1], rxd[2]);
}

void read_reg_all()
{
    uint8_t address = 0x20;
    for(int i=1;i<=24;i++){
        read_reg(address);
        address += 1;
    }
}

void setup(){
    send_command(CMD_SDATAC);
}

void initialize(){
    write_reg(0x01, 0x92); //4k
    
    write_reg(0x02, 0xC0);
    write_reg(0x03, 0xE0);
    write_reg(0x04, 0x00);

    write_reg(0x05, 0x60);
    write_reg(0x06, 0x60);
    write_reg(0x07, 0x60);
    write_reg(0x08, 0x60);
    write_reg(0x09, 0x60);
    write_reg(0x0A, 0x60);
    write_reg(0x0B, 0x60);
    write_reg(0x0C, 0x60);

    write_reg(0x0D, 0x00);
    write_reg(0x0E, 0x00);
    write_reg(0x0F, 0x00);
    write_reg(0x10, 0x00);
    write_reg(0x11, 0x00);
    // 0x12 and 0x13 are read-only Reg
    write_reg(0x14, 0x00);
    write_reg(0x15, 0x20); // enable SRB1
    write_reg(0x16, 0x00);
    write_reg(0x17, 0x00);
}

void test_signal(){
    write_reg(0x02, 0xD0); // internel test signal

    write_reg(0x05, 0x65);
    write_reg(0x06, 0x65);
    write_reg(0x07, 0x65);
    write_reg(0x08, 0x65);
    write_reg(0x09, 0x65);
    write_reg(0x0A, 0x65);
    write_reg(0x0B, 0x65);
    write_reg(0x0C, 0x65);
}

void change_sample_rate(int sample_rate)
{
    switch(sample_rate)
    {
        case 4000:
            write_reg(0x01, 0x92); //4k
            break;
        case 2000:
            write_reg(0x01, 0x93);
            break;
        case 1000:
            write_reg(0x01, 0x94);
            break;
        case 500:
            write_reg(0x01, 0x95);
            break;
        case 250:
            write_reg(0x01, 0x96);
            break;
        default:
            write_reg(0x01, 0x92); // 4K
            break;
    }
}

void change_mux(int mux)
{
    uint8_t write_bits;
    switch(mux)
    {
        case TEST_SIGNAL:
            write_reg(0x02, 0xD0); 
            write_bits = 0x05;
            break;
        case INPUT_SHORTED:
            write_bits = 0x01;
            break;
        case NORMAL_ELEC_INPUT:
            write_reg(0x03, 0xEC);
            write_bits = 0x00;
            break;
        default: 
            write_reg(0x02, 0xD0); 
            write_bits = 0x05;
            break;
    }
    
    // 逐个通道写入
    write_reg(0x05, 0x60|write_bits);
    write_reg(0x06, 0x60|write_bits);
    write_reg(0x07, 0x60|write_bits);
    write_reg(0x08, 0x60|write_bits);
    write_reg(0x09, 0x60|write_bits);
    write_reg(0x0A, 0x60|write_bits);
    write_reg(0x0B, 0x60|write_bits);
    write_reg(0x0C, 0x60|write_bits);
}

// -------- RPI -------- //

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    terminate(0);
}

// Map GPIO, DMA and SPI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&spi_regs, (void *)SPI0_BASE, PAGE_SIZE);
    map_periph(&clk_regs, (void *)CLK_BASE, PAGE_SIZE);
    map_periph(&pwm_regs, (void *)PWM_BASE, PAGE_SIZE);
    // map_periph(&usec_regs, (void *)USEC_BASE, PAGE_SIZE);
}

// Free memory & peripheral mapping and exit
void terminate(int sig)
{
    printf("Closing\n");
    spi_disable();
    stop_dma(DMA_CHAN_A);
    stop_dma(DMA_CHAN_B);
    unmap_periph_mem(&usec_regs);
    unmap_periph_mem(&pwm_regs);
    unmap_periph_mem(&clk_regs);
    unmap_periph_mem(&spi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    if (fifo_name)
        destroy_fifo(fifo_name, fifo_fd);
    unmap_periph_mem(&vc_mem);
    // if (samp_total)
    //     printf("Total samples %u, overruns %u\n", samp_total, overrun_total);
    exit(0);
}


// ————————————————SPI———————————————— //

// Initialise SPI0, given desired clock freq; return actual value
int init_spi(int hz)
{
    int f, div = (SPI_CLOCK_HZ / hz + 1) & ~1;

    gpio_set(SPI0_CE0_PIN, GPIO_ALT0, GPIO_NOPULL);
    // gpio_set(SPI0_CE1_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_MISO_PIN, GPIO_ALT0, GPIO_PULLUP);
    gpio_set(SPI0_MOSI_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_SCLK_PIN, GPIO_ALT0, GPIO_NOPULL);
    while (div==0 || (f = SPI_CLOCK_HZ/div) > MAX_SPI_FREQ)
        div += 2;
    *REG32(spi_regs, SPI_CS) = 0x34;
    *REG32(spi_regs, SPI_CLK) = div;
    return(f);
}

// Clear SPI FIFOs
void spi_clear(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
}

// Set / clear SPI chip select
void spi_cs(int set)
{
    uint32_t csval = *REG32(spi_regs, SPI_CS);

    *REG32(spi_regs, SPI_CS) = set ? csval | 0x80 : csval & ~0x80;
}

// Transfer SPI bytes
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len)
{
    while (len--)
    {
        *REG8(spi_regs, SPI_FIFO) = *txd++;
        while((*REG32(spi_regs, SPI_CS) & (1<<17)) == 0) ;
        *rxd++ = *REG32(spi_regs, SPI_FIFO);
    }
}

// Disable SPI
void spi_disable(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
    *REG32(spi_regs, SPI_CS) = 0;
}

// Display SPI registers
void disp_spi(void)
{
    volatile uint32_t *p=REG32(spi_regs, SPI_CS);
    int i=0;

    while (spi_regstrs[i][0])
        printf("%-4s %08X ", spi_regstrs[i++], *p++);
    printf("\n");
}


// -------- FIFO -------- //
// Create a FIFO (named pipe)
int create_fifo(char *fname)
{
    int ok=0;

    umask(0);
    if (mkfifo(fname, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH) < 0 && errno != EEXIST)
        printf("Can't open FIFO '%s'\n", fname);
    else
        ok = 1;
    return(ok);
}

// Open a FIFO for writing, return 0 if there is no reader
int open_fifo_write(char *fname)
{
    int f = open(fname, O_WRONLY); 

    return(f == -1 ? 0 : f);
}

// Write to FIFO, return 0 if no reader
int write_fifo(int fd, void *data, int dlen)
{
    struct pollfd pollfd = {fd, POLLOUT, 0};

    poll(&pollfd, 1, -1);

    if (pollfd.revents&POLLOUT && !(pollfd.revents&POLLERR))
    {
        int write_num = write(fd, data, dlen);
        // printf("fd:%d, write_num:%d \n", fd, write_num);
        return (fd ? write_num : 0);
        // return(fd ? write(fd, data, dlen) : 0); // If successful, write() returns the number of bytes written
    }
    return(0);
}

// Return the free space in FIFO
uint32_t fifo_freespace(int fd)
{
    return(fcntl(fd, F_GETPIPE_SZ)); 
}

// Remove the FIFO
void destroy_fifo(char *fname, int fd)
{
    if (fd > 0)
        close(fd);
    unlink(fname);
}