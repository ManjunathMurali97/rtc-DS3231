#include <linux/acpi.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc/ds1307.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#define DS3231_REG_SECS			0x00
#	define DS3231_BIT_CH			0x80
#define DS3231_REG_MIN			0x01
#define DS3231_REG_HOUR			0x02
#	define DS3231_BIT_12HR		0x40
#	define DS3231_BIT_PM			0x20
#define DS3231_REG_WDAY			0x03
#define DS3231_REG_MDAY			0x04
#define DS3231_REG_MONTH		0x05
#	define DS3231_BIT_CENTURY	0x80
#define DS3231_REG_YEAR			0x06

#define DS3231_REG_CONTROL 	0x0e
#	define DS3231_BIT_EOSC	  0x80
#	define DS3231_BIT_BBSQW		0x40

#define DS3231_REG_STATUS		0x0f
#	define DS3231_BIT_OSF			0x80


/////////////////////////////////////////////////////
static const unsigned short normal_i2c[] = { 0x68 , I2C_CLIENT_END };


struct ds3231 {
	struct device 		*dev;
	struct regmap 		*regmap;
	struct rtc_device 	*rtc; /* An RTC device is represented in the kernel as an instance of the struct rtc_device structure */
	const char 		*name;
};

struct chip_desc {
	unsigned		alarm:1;
	u16			nvram_offset;
	u16			nvram_size;
	u8			offset; /* register's offset */
	u8			century_reg;
	u8			century_enable_bit;
	u8			century_bit;
	u8			bbsqi_bit;
	irq_handler_t		irq_handler;
	const struct rtc_class_ops *rtc_ops;
	u16			trickle_charger_reg;
	u8			(*do_trickle_setup)(struct ds3231 *, u32,
			bool);
};

static const struct chip_desc chips={
		.alarm		= 1,
		.century_reg	= DS3231_REG_MONTH,
		.century_bit	= DS3231_BIT_CENTURY,
		.bbsqi_bit		= DS3231_BIT_BBSQW,
};

static const struct regmap_config regmap_config = {
	.reg_bits=8,
	.val_bits=8,
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int ds3231_get_time(struct device *dev , struct rtc_time *t)
{
	struct ds3231 *ds3231=dev_get_drvdata(dev); /*refer probe function where we are setting driver data*/
	u8 regs[7];
	int tmp,ret;
	const struct chip_desc *chip = &chips;

	ret = regmap_bulk_read( ds3231->regmap , chip->offset , regs , sizeof(regs) );
	if (ret) {
		dev_err(dev, "%s error %d\n", "read", ret);
		return ret;
	}
	dev_dbg(dev, "%s: %7ph\n", "read", regs);

	tmp = regs[DS3231_REG_SECS];
	if(tmp & DS3231_BIT_CH)
		return -EINVAL;

	t->tm_sec = bcd2bin(regs[DS3231_REG_SECS] & 0x7f);
	t->tm_min = bcd2bin(regs[DS3231_REG_MIN] & 0x7f);
	tmp = regs[DS3231_REG_HOUR] & 0x3f;
	t->tm_hour = bcd2bin(tmp);
	t->tm_wday = bcd2bin(regs[DS3231_REG_WDAY] & 0x07) - 1;
	t->tm_mday = bcd2bin(regs[DS3231_REG_MDAY] & 0x3f);
	tmp = regs[DS3231_REG_MONTH] & 0x1f;
	t->tm_mon = bcd2bin(tmp) - 1;
	t->tm_year = bcd2bin(regs[DS3231_REG_YEAR]) + 100;

	if (regs[chip->century_reg] & chip->century_bit &&
			IS_ENABLED(CONFIG_RTC_DRV_DS1307_CENTURY))
		t->tm_year += 100;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
			"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
			"read", t->tm_sec, t->tm_min,
			t->tm_hour, t->tm_mday,
			t->tm_mon, t->tm_year, t->tm_wday);

	return 0;
}


static int ds3231_set_time(struct device *dev, struct rtc_time *t)
{
	struct ds3231	*ds3231 = dev_get_drvdata(dev);
	const struct chip_desc *chip = &chips;
	int		result;
	int		tmp;
	u8		regs[7];

	dev_dbg(dev, "%s secs=%d, mins=%d, "
			"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
			"write", t->tm_sec, t->tm_min,
			t->tm_hour, t->tm_mday,
			t->tm_mon, t->tm_year, t->tm_wday);

	if (t->tm_year < 100)
		return -EINVAL;

	regs[DS3231_REG_SECS] = bin2bcd(t->tm_sec);
	regs[DS3231_REG_MIN] = bin2bcd(t->tm_min);
	regs[DS3231_REG_HOUR] = bin2bcd(t->tm_hour);
	regs[DS3231_REG_WDAY] = bin2bcd(t->tm_wday + 1);
	regs[DS3231_REG_MDAY] = bin2bcd(t->tm_mday);
	regs[DS3231_REG_MONTH] = bin2bcd(t->tm_mon + 1);

	/* assume 20YY not 19YY */
	tmp = t->tm_year - 100;
	regs[DS3231_REG_YEAR] = bin2bcd(tmp);

	dev_dbg(dev, "%s: %7ph\n", "write", regs);

	result = regmap_bulk_write(ds3231->regmap, chip->offset, regs,
			sizeof(regs));
	if (result) {
		dev_err(dev, "%s error %d\n", "write", result);
		return result;
	}

	return 0;
}


static const struct rtc_class_ops ds3231_rtc_ops = {
	.read_time	=	ds3231_get_time ,
	.set_time	=	ds3231_set_time ,
};


static const struct i2c_device_id ds3231_i2c_id[] = {
	{"ds3231", 0 },
	{ }
};
MODULE_DEVICE_TABLE( i2c , ds3231_i2c_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id ds3231_acpi_ids[] = {
	{ .id = "DS3231", .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE( acpi , ds3231_acpi_ids);
#endif

#ifdef CONFIG_OF
static const struct of_device_id ds3231_of_match[] = {
	{
		.compatible = "maxim,ds3231",
		.data = (void *)0,
	},
	{ }
};
MODULE_DEVICE_TABLE( of , ds3231_of_match );
#endif

/////////////////////////////////////////////////////////////////////////////////
static int ds3231_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct ds3231 	*ds3231;
	unsigned int 	regs[8];
	int 		err= -ENODEV;
	const struct chip_desc *chip;
	ds3231 = devm_kzalloc( &client->dev , sizeof(struct ds3231) , GFP_KERNEL);
	if(!ds3231)
		return -ENOMEM;

	dev_set_drvdata( &client->dev , ds3231 );
	ds3231->dev = &client->dev;
	ds3231->name = client->name;

	ds3231->regmap = devm_regmap_init_i2c( client , &regmap_config);
	if (IS_ERR(ds3231->regmap)) {
		dev_err(ds3231->dev, "regmap allocation failed\n");
		return PTR_ERR(ds3231->regmap);
	}

	i2c_set_clientdata( client , ds3231);

	/* Reading Control and Status registers of rtc*/
	err = regmap_bulk_read( ds3231->regmap , DS3231_REG_CONTROL , regs , 2 );
	if(err){
		dev_dbg( ds3231->dev , "Read error %d\n" , err);
		return err;
	}

	/* If oscillator is off , turn it on , so clock can tick */
	if( regs[0] & DS3231_BIT_EOSC)
		regs[0] &= ~DS3231_BIT_EOSC;

	regmap_write( ds3231->regmap , DS3231_REG_CONTROL , regs[0] );

	/*oscillator fault ? clear flag and warn */
	if( regs[1] & DS3231_BIT_OSF){
		regmap_write( ds3231->regmap , DS3231_REG_STATUS , regs[1] & ~DS3231_BIT_OSF);
		dev_warn( ds3231->dev , "SET TIME!\n");
	}

	/* read RTC registers */
	err = regmap_bulk_read( ds3231->regmap , chip->offset , regs , sizeof(regs) );
	if (err) {
		dev_dbg(ds3231->dev, "read error %d\n", err);
		return err;
	}
	return 0;
}



static int ds3231_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter= client->adapter;
	int address = client->addr;
	const char *name = NULL;

	if(!i2c_check_functionality(adapter,I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if(address == 0x68)
	{
		name = "ds3231";
		dev_info( &adapter->dev , "rtc device found at 0x%02x\n" , address );
	}
	else
		return -ENODEV;

	strlcpy( info->type , name , I2C_NAME_SIZE );
	return 0;
}



struct i2c_driver ds3231_driver={
	.class =	I2C_CLASS_HWMON,
	.driver={
		.name			= "rtc-ds3231",
		.of_match_table		= of_match_ptr(ds3231_of_match),
		.acpi_match_table 	= ACPI_PTR(ds3231_acpi_ids),
	},
	.probe				=	ds3231_probe,
	.id_table			=	ds3231_i2c_id,
	.detect 			=	ds3231_detect,
	.address_list = normal_i2c,
};



module_i2c_driver(ds3231_driver);
MODULE_DESCRIPTION("RTC driver for DS3231");
MODULE_LICENSE("GPL");
