#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define DEV_NAME "lcd"
#define I2C_BUS_NUM             1       // I2C-1 버스 사용
#define LCD_ADDR                0x27    // 디바이스 슬레이브 주소 <- sudo i2cdetect -y 1 로 확인

#define EMPTY_LINE              "                "

static void lcd_init(void);
static void lcd_command(uint8_t command, uint8_t is_init);
static void lcd_data(uint8_t data);
static void lcd_string(char *str);
static void move_cursor(uint8_t row,uint8_t column);

static struct i2c_adapter *i2c_adap;
static struct i2c_client *i2c_client;
static int major_num;

static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{
    char data[18] = {0};

    if (len < 1) return -EINVAL;
    if (len > 18) return -EINVAL;

    if (copy_from_user(data, buf, len-1)) return -EFAULT;


        if (data[0] == '1')
        {
                move_cursor(0,0);
                lcd_string(EMPTY_LINE);
                move_cursor(0,0);
        }
        else if (data[0] == '2')
        {
                move_cursor(1,0);
                lcd_string(EMPTY_LINE);
                move_cursor(1,0);
        }
        else return 0;

        lcd_string(data+1);
        pr_info("line %c : %s\n", data[0], data+1);
        return len;
}

void lcd_init(void){
        msleep(50);
        lcd_command(0x33, 1);
        udelay(150);
        lcd_command(0x32, 0);
        lcd_command(0x28, 0);
        lcd_command(0x0c, 0);
        lcd_command(0x06, 0);
        lcd_command(0x01, 0);
        msleep(2);
}

void lcd_command(uint8_t command, uint8_t is_init){
        uint8_t high_nibble, low_nibble;
        uint8_t i2c_buffer[4];
        high_nibble = command & 0xf0;
        low_nibble = (command<<4) & 0xf0;
        i2c_buffer[0] = high_nibble | 0x04 | 0x08;
        i2c_buffer[1] = high_nibble | 0x00 | 0x08;
        if (is_init) msleep(5);
        i2c_buffer[2] = low_nibble  | 0x04 | 0x08;
        i2c_buffer[3] = low_nibble  | 0x00 | 0x08;
        i2c_master_send(i2c_client, i2c_buffer, 4);
}

void lcd_data(uint8_t data){
        uint8_t high_nibble, low_nibble;
        uint8_t i2c_buffer[4];
        high_nibble = data & 0xf0;
        low_nibble = (data<<4) & 0xf0;
        i2c_buffer[0] = high_nibble | 0x05 | 0x08;
        i2c_buffer[1] = high_nibble | 0x01 | 0x08;
        i2c_buffer[2] = low_nibble  | 0x05 | 0x08;
        i2c_buffer[3] = low_nibble  | 0x01 | 0x08;
        i2c_master_send(i2c_client, i2c_buffer, 4);
}

void lcd_string(char *str){
        while(*str) lcd_data(*str++);
}

void move_cursor(uint8_t row, uint8_t column){
        lcd_command(0x80 | row<<6 | column, 0);
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .write = dev_write,
};

static int __init init_system(void)
{
    struct i2c_board_info board_info = {
        I2C_BOARD_INFO("lcd", LCD_ADDR)
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

        lcd_init();

    major_num = register_chrdev(0, DEV_NAME, &fops);
    if (major_num < 0) {
        pr_err("Device registration failed\n");
        i2c_unregister_device(i2c_client);
        i2c_put_adapter(i2c_adap);
                return major_num;
        }
    pr_info("Major number: %d\n", major_num);

    return 0;
}

static void __exit exit_system(void)
{
    i2c_unregister_device(i2c_client);
    i2c_put_adapter(i2c_adap);
    pr_info("lcd removed\n");
}

module_init(init_system);
module_exit(exit_system);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wodud");
MODULE_DESCRIPTION("LCD I2C Driver");
