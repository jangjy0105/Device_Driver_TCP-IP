#include <linux/module.h>    // 모든 커널 모듈에 필수
#include <linux/kernel.h>    // pr_info(), pr_err() 등 커널 로깅 함수
#include <linux/fs.h>        // file_operations 구조체 및 캐릭터 디바이스 관리 함수
#include <linux/uaccess.h>   // copy_to_user(): 커널 공간 -> 사용자 공간 데이터 복사
#include <linux/i2c.h>       // I2C 서브시스템 관련 함수
#include <linux/delay.h>     // msleep() 함수
#include <linux/slab.h>      // kzalloc(): 커널 메모리 할당 함수
#include <linux/swab.h>      // swab16() 매크로를 사용하기 위해 필요

/* 디바이스 및 I2C 관련 상수 정의 */
#define DEVICE_NAME "bmp180"            // 등록할 디바이스 이름
#define I2C_BUS_AVAILABLE 1             // 사용할 I2C 버스 번호 (라즈베리 파이는 보통 1번)
#define BMP180_I2C_ADDR 0x77            // BMP180 센서의 I2C 주소

/* BMP180 레지스터 주소 정의 */
#define BMP180_REG_CAL_AC1      0xAA    // 보정 데이터 시작 주소
#define BMP180_REG_CONTROL      0xF4    // 제어 레지스터
#define BMP180_REG_RESULT       0xF6    // 결과 레지스터
#define BMP180_CMD_READ_TEMP    0x2E    // 온도 측정 명령
#define BMP180_CMD_READ_PRESSURE 0x34   // 기압 측정 명령 (oss=0)

/* 전역 변수 선언 */
static struct i2c_client *bmp180_client; // I2C 클라이언트를 가리키는 포인터
static int major_number;                 // 우리 드라이버에 할당될 Major 번호

/* 보정 데이터를 저장할 구조체 */
struct bmp180_cal_param {
    short ac1, ac2, ac3, b1, b2, mb, mc, md;
    unsigned short ac4, ac5, ac6;
};
static struct bmp180_cal_param bmp180_cal; // 보정 데이터 저장용 전역 변수


/*
 * 함수 원형 선언
 */
static int bmp180_open(struct inode *inode, struct file *file);
static int bmp180_release(struct inode *inode, struct file *file);
static ssize_t bmp180_read(struct file *file, char __user *buf, size_t len, loff_t *off);
static int bmp180_get_cal_param(void);
static long bmp180_get_ut(void);
static long bmp180_get_up(void);


/*
 * 사용자가 파일을 open, read, close 할 때 호출될 함수들을 모아놓은 구조체
 */
static const struct file_operations bmp180_fops = {
    .owner   = THIS_MODULE,
    .open    = bmp180_open,
    .read    = bmp180_read,
    .release = bmp180_release,
};

/**
 * bmp180_get_cal_param - 센서의 EEPROM에서 보정 데이터
 */
static int bmp180_get_cal_param(void) {
    char cal_data[22];
    int ret;

    ret = i2c_smbus_read_i2c_block_data(bmp180_client, BMP180_REG_CAL_AC1, 22, cal_data);
    if (ret != 22) {
        pr_err("BMP180: Failed to read calibration data. ret=%d\n", ret);
        return -EIO;
    }

    bmp180_cal.ac1 = (cal_data[0] << 8) | cal_data[1];
    bmp180_cal.ac2 = (cal_data[2] << 8) | cal_data[3];
    bmp180_cal.ac3 = (cal_data[4] << 8) | cal_data[5];
    bmp180_cal.ac4 = (cal_data[6] << 8) | cal_data[7];
    bmp180_cal.ac5 = (cal_data[8] << 8) | cal_data[9];
    bmp180_cal.ac6 = (cal_data[10] << 8) | cal_data[11];
    bmp180_cal.b1  = (cal_data[12] << 8) | cal_data[13];
    bmp180_cal.b2  = (cal_data[14] << 8) | cal_data[15];
    bmp180_cal.mb  = (cal_data[16] << 8) | cal_data[17];
    bmp180_cal.mc  = (cal_data[18] << 8) | cal_data[19];
    bmp180_cal.md  = (cal_data[20] << 8) | cal_data[21];

    pr_info("BMP180: Calibration data read successfully.\n");
    pr_info("AC1=%d, AC2=%d, MC=%d, MD=%d\n", bmp180_cal.ac1, bmp180_cal.ac2, bmp180_cal.mc, bmp180_cal.md);

    return 0;
}

/**
 * bmp180_get_ut - 보정되지 않은 온도 값
 */
static long bmp180_get_ut(void) {
    s32 ret;
    i2c_smbus_write_byte_data(bmp180_client, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP);
    msleep(5);
    // i2c_smbus_read_word_be 대신 read_word_data와 swab16 사용
    ret = i2c_smbus_read_word_data(bmp180_client, BMP180_REG_RESULT);
    if (ret < 0)
        return ret;
    return swab16(ret);
}

/**
 * bmp180_get_up - 보정되지 않은 기압 값
 */
static long bmp180_get_up(void) {
    s32 ret;
    i2c_smbus_write_byte_data(bmp180_client, BMP180_REG_CONTROL, BMP180_CMD_READ_PRESSURE);
    msleep(5);
    // i2c_smbus_read_word_be 대신 read_word_data와 swab16 사용 
 
    ret = i2c_smbus_read_word_data(bmp180_client, BMP180_REG_RESULT);
    if (ret < 0)
        return ret;
    return swab16(ret);
}

/**
 * bmp180_open - 사용자가 /dev/bmp180 파일을 열 때 호출.
 */
static int bmp180_open(struct inode *inode, struct file *file) {
    pr_info("BMP180: Device opened.\n");
    return 0;
}

/**
 * bmp180_release - 사용자가 /dev/bmp180 파일을 닫을 때 호출.
 */
static int bmp180_release(struct inode *inode, struct file *file) {
    pr_info("BMP180: Device released.\n");
    return 0;
}

/**
 * bmp180_read - 사용자가 /dev/bmp180 파일에서 데이터를 읽을 때 호출
 */
static ssize_t bmp180_read(struct file *file, char __user *buf, size_t len, loff_t *off) {
    long ut, up;
    long x1, x2, b5, b6, x3, b3, p;
    unsigned long b4, b7;
    long temp, press;

    char result_str[100];
    int str_len;

    // 이미 한 번 데이터를 읽었다면, 파일의 끝(EOF)임을 알리기 위해 0을 반환
   
    if (*off > 0) {
        return 0;
    }

    pr_info("BMP180: Reading data from sensor...\n");

    /* 1. 온도 계산 */
    ut = bmp180_get_ut();
    if (ut < 0) return ut;
    x1 = ((ut - bmp180_cal.ac6) * bmp180_cal.ac5) >> 15;
    if (unlikely(x1 + bmp180_cal.md == 0)) {
        pr_warn("BMP180: Division by zero in temp calculation.\n");
        return -EREMOTEIO;
    }
    x2 = (bmp180_cal.mc << 11) / (x1 + bmp180_cal.md);
    b5 = x1 + x2;
    temp = (b5 + 8) >> 4;

    /* 2. 기압 계산 */
    up = bmp180_get_up();
    if (up < 0) return up;
    b6 = b5 - 4000;
    x1 = (bmp180_cal.b2 * (b6 * b6 >> 12)) >> 11;
    x2 = bmp180_cal.ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((long)bmp180_cal.ac1 * 4 + x3) + 2) >> 2;
    x1 = bmp180_cal.ac3 * b6 >> 13;
    x2 = (bmp180_cal.b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (bmp180_cal.ac4 * (unsigned long)(x3 + 32768)) >> 15;
    if (unlikely(b4 == 0)) {
        pr_warn("BMP180: Division by zero in pressure calculation (b4=0).\n");
        return -EREMOTEIO;
    }
    b7 = ((unsigned long)up - b3) * 50000;

    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    press = p + ((x1 + x2 + 3791) >> 4);

    // 온도: 0.1 C 단위이므로 10으로 나누어 정수부와 소수부를 표현
    // 압력: Pa 단위이므로 100으로 나누어 hPa(헥토파스칼)로 표현
    sprintf(result_str, "Temperature: %ld.%ld C, Pressure: %ld hPa\n",
            temp / 10, temp % 10, press / 100);

    str_len = strlen(result_str);

    if (copy_to_user(buf, result_str, str_len)) {
        pr_err("BMP180: Failed to copy data to user.\n");
        return -EFAULT;
    }

    pr_info("BMP180: Formatted string sent to user.\n");

    // 사용자에게 몇 바이트를 전송했는지 알림
    *off += str_len;

    return str_len; // 전송한 문자열의 길이를 반환
}

/**
 * bmp180_init - 모듈이 커널에 로드될 때 호출되는 초기화 함수
 */
static int __init bmp180_init(void) {
    struct i2c_adapter *i2c_adap;

    pr_info("BMP180: Initializing the BMP180 driver...\n");

    major_number = register_chrdev(0, DEVICE_NAME, &bmp180_fops);
    if (major_number < 0) {
        pr_err("BMP180: Failed to register a major number\n");
        return major_number;
    }
    pr_info("BMP180: Registered with major number %d\n", major_number);

    i2c_adap = i2c_get_adapter(I2C_BUS_AVAILABLE);
    if (i2c_adap == NULL) {
        pr_err("BMP180: Failed to get I2C adapter\n");
        unregister_chrdev(major_number, DEVICE_NAME);
        return -ENODEV;
    }

    bmp180_client = i2c_new_client_device(i2c_adap, &(struct i2c_board_info){ I2C_BOARD_INFO(DEVICE_NAME, BMP180_I2C_ADDR) });
    if (IS_ERR(bmp180_client)) { // i2c_new_client_device는 이제 NULL 대신 ERR_PTR을 반환
        pr_err("BMP180: Failed to create I2C client\n");
        i2c_put_adapter(i2c_adap);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(bmp180_client);
    }

    i2c_put_adapter(i2c_adap);

    if (bmp180_get_cal_param() != 0) {
        pr_err("BMP180: Failed to get calibration data, aborting.\n");
        i2c_unregister_device(bmp180_client);
        unregister_chrdev(major_number, DEVICE_NAME);
        return -EIO;
    }

    pr_info("BMP180: Driver initialization complete.\n");
    return 0;
}

/**
 * bmp180_exit - 모듈이 커널에서 언로드될 때 호출되는 종료 함수
 */
static void __exit bmp180_exit(void) {
    i2c_unregister_device(bmp180_client);
    pr_info("BMP180: Unregistered I2C client.\n");

    unregister_chrdev(major_number, DEVICE_NAME);
    pr_info("BMP180: Unregistered character device.\n");

    pr_info("BMP180: Driver exit complete.\n");
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("KIM");
MODULE_DESCRIPTION("BMP180 temperature and pressure sensor I2C driver");
MODULE_VERSION("0.3");
