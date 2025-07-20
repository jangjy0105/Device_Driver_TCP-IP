#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define DEV_NAME "bmp180"
#define I2C_BUS_NUM             1       // I2C-1 버스 사용
#define BMP180_ADDR             0x77    // 디바이스 슬레이브 주소 <- sudo i2cdetect -y 1 로 확인

#define OSS                             1

static void get_ut(void);
static void get_up(void);
static void calculate_t(void);
static void calculate_p(void);

static struct i2c_adapter *i2c_adap;
static struct i2c_client *i2c_client;
static int major_num;


struct bmp180_t{
        short                   ac1;
        short                   ac2;
        short                   ac3;
        unsigned short  ac4;
        unsigned short  ac5;
        unsigned short  ac6;
        short                   b1;
        short                   b2;
        long                    b3;
        unsigned long   b4;
        long                    b5;
        long                    b6;
        unsigned long   b7;
        short                   mb;
        short                   mc;
        short                   md;
        long                    x1;
        long                    x2;
        long                    x3;
        long                    ut;
        long                    up;
        long                    t;
        long                    p;
};

struct bmp180_t bmp180;

static ssize_t dev_read(struct file *file, char __user *buf, size_t len, loff_t *offset)
{
    char msg[32] = "";

        if (*offset > 0) return 0;

        get_ut();
        get_up();
        calculate_t();
        calculate_p();

    sprintf(msg, "T = %ld.%ld, P = %ld\n", bmp180.t/10, bmp180.t%10, bmp180.p);

    *offset = 0;

    return simple_read_from_buffer(buf, len, offset, msg, strlen(msg) + 1);
}

void get_ut(void){
        uint8_t write_buf[2] = { 0xf4, 0x2e };
        uint8_t data_reg = 0xf6;
        uint8_t read_buf[2];

        i2c_master_send(i2c_client, write_buf, 2);
        msleep(5);

        i2c_master_send(i2c_client, &data_reg, 1);
        i2c_master_recv(i2c_client, read_buf, 2);

        bmp180.ut = ( read_buf[0] << 8 ) + read_buf[1];
}

void get_up(void){
        uint8_t write_buf[2] = {0xf4, 0x34+(OSS<<6)};
        uint8_t data_reg = 0xf6;
        uint8_t read_buf[3];

        i2c_master_send(i2c_client, write_buf, 2);
        msleep(8);

        i2c_master_send(i2c_client, &data_reg, 1);
        i2c_master_recv(i2c_client, read_buf, 3);
        bmp180.up = ( (read_buf[0]<<16) + (read_buf[1]<<8) + (read_buf[2]) ) >> (8-OSS);
}

void calculate_t(void){
        bmp180.x1 = (bmp180.ut - bmp180.ac6) * bmp180.ac5 / (1<<15);
        bmp180.x2 = (bmp180.mc * (1<<11)) / (bmp180.x1 + bmp180.md);
        bmp180.b5 = bmp180.x1 + bmp180.x2;
        bmp180.t = (bmp180.b5 + 8) / (1<<4);
}

void calculate_p(void){
        bmp180.b6 = bmp180.b5 - 4000;
        bmp180.x1 = ( bmp180.b2 * (bmp180.b6*bmp180.b6/(1<<12)) ) / (1<<11);
        bmp180.x2 = bmp180.ac2 * bmp180.b6 / (1<<11);
        bmp180.x3 = bmp180.x1 + bmp180.x2;
        bmp180.b3 = ( ( (bmp180.ac1*4 + bmp180.x3)<<OSS ) + 2 ) / 4;
        bmp180.x1 = bmp180.ac3 * bmp180.b6 / (1<<13);
        bmp180.x2 = ( bmp180.b1 * ( bmp180.b6*bmp180.b6/(1<<12) ) ) / (1<<16);
        bmp180.x3= ( (bmp180.x1 + bmp180.x2) + 2 ) / (1<<2);
        bmp180.b4 = bmp180.ac4 * (unsigned long)(bmp180.x3 + 32768) / (1<<15);
        bmp180.b7 = ( (unsigned long)bmp180.up - bmp180.b3 ) * (50000>>OSS);
        if(bmp180.b7 < 0x80000000)      bmp180.p = (bmp180.b7 * 2) / bmp180.b4;
        else                                            bmp180.p = ( bmp180.b7 / bmp180.b4 ) * 2;
        bmp180.x1 = (bmp180.p/(1<<8)) * (bmp180.p/(1<<8));
        bmp180.x1 = ( bmp180.x1 * 3038 ) / (1<<16);
        bmp180.x2 = (-7357 * bmp180.p) / (1<<16);
        bmp180.p = bmp180.p + (bmp180.x1 + bmp180.x2 + 3791)/(1<<4);
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = dev_read,
};

static int __init bmp180_init(void)
{
        msleep(1000);
        uint8_t start_addr[] = { 0xaa };
        uint8_t read_data[22];
    struct i2c_board_info board_info = {
        I2C_BOARD_INFO("bmp180", BMP180_ADDR)
    };

    // 1. I2C 어댑터 획득
    i2c_adap = i2c_get_adapter(I2C_BUS_NUM);
    if (!i2c_adap) {
        pr_err("I2C adapter not found\n");
        return -ENODEV;
    }

    // 2. I2C 클라이언트 생성
    i2c_client = i2c_new_client_device(i2c_adap, &board_info);
    if (!i2c_client) {
        pr_err("Device registration failed\n");
        i2c_put_adapter(i2c_adap);
        return -ENODEV;
    }

    i2c_master_send(i2c_client, start_addr, sizeof(start_addr));
        i2c_master_recv(i2c_client, read_data, 22);
        bmp180.ac1 = (read_data[0] << 8) | read_data[1];
        bmp180.ac2 = (read_data[2] << 8) | read_data[3];
        bmp180.ac3 = (read_data[4] << 8) | read_data[5];
        bmp180.ac4 = (read_data[6] << 8) | read_data[7];
        bmp180.ac5 = (read_data[8] << 8) | read_data[9];
        bmp180.ac6 = (read_data[10] << 8) | read_data[11];
        bmp180.b1 = (read_data[12] << 8) | read_data[13];
        bmp180.b2 = (read_data[14] << 8) | read_data[15];
        bmp180.mb = (read_data[16] << 8) | read_data[17];
        bmp180.mc = (read_data[18] << 8) | read_data[19];
        bmp180.md = (read_data[20] << 8) | read_data[21];

    major_num = register_chrdev(0, DEV_NAME, &fops);
    if (major_num < 0) {
        pr_err("Device registration failed\n");
        return major_num;
        }
    pr_info("Major number: %d\n", major_num);

    return 0;
}

static void __exit bmp180_exit(void)
{
    i2c_unregister_device(i2c_client);
    i2c_put_adapter(i2c_adap);
    pr_info("BMP180 removed\n");
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wodud");
MODULE_DESCRIPTION("BMP180 I2C Driver");
