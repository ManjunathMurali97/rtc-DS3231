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

#define DS3231_REG_SECS		0x00
#	define DS1307_BIT_CH		0x80
#define DS3231_REG_MIN		0x01
#define DS3231_REG_HOUR		0X02
#	define DS3231_BIT_12HR	0X40
#	define DS3231_BIT_PM	0X20
#define DS3231_REG_DAY		0X03
#define DS3231_REG_DATE		0X04
#define DS3231_REG_MONTH	0x05
#define DS3231_REG_YEAR		0X06


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

static const struct chip_desc chips;

static const struct regmap_config regmap_config = {
	.reg_bits=8;
	.val_bits=8;
};

static const struct i2c_device_id ds3231_i2c_id[] = {
	{"ds3231", ds3231}
	{}
};
MODULE_DEVICE_TABLE( i2c , ds3231_i2c_id);





//////////////////////////////////////////////////////////////



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

	t->tm_sec = bcd2bin(regs[DS1307_REG_SECS] & 0x7f);
	t->tm_min = bcd2bin(regs[DS1307_REG_MIN] & 0x7f);
	tmp = regs[DS1307_REG_HOUR] & 0x3f;
	t->tm_hour = bcd2bin(tmp);
	t->tm_wday = bcd2bin(regs[DS1307_REG_WDAY] & 0x07) - 1;
	t->tm_mday = bcd2bin(regs[DS1307_REG_MDAY] & 0x3f);
	tmp = regs[DS1307_REG_MONTH] & 0x1f;
	t->tm_mon = bcd2bin(tmp) - 1;
	t->tm_year = bcd2bin(regs[DS1307_REG_YEAR]) + 100;

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

	regs[DS1307_REG_SECS] = bin2bcd(t->tm_sec);
	regs[DS1307_REG_MIN] = bin2bcd(t->tm_min);
	regs[DS1307_REG_HOUR] = bin2bcd(t->tm_hour);
	regs[DS1307_REG_WDAY] = bin2bcd(t->tm_wday + 1);
	regs[DS1307_REG_MDAY] = bin2bcd(t->tm_mday);
	regs[DS1307_REG_MONTH] = bin2bcd(t->tm_mon + 1);

	/* assume 20YY not 19YY */
	tmp = t->tm_year - 100;
	regs[DS1307_REG_YEAR] = bin2bcd(tmp);

	dev_dbg(dev, "%s: %7ph\n", "write", regs);

	result = regmap_bulk_write(ds1307->regmap, chip->offset, regs,
			sizeof(regs));
	if (result) {
		dev_err(dev, "%s error %d\n", "write", result);
		return result;
	}

	return 0;


}




static int ds3231_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct ds3231 	*ds3231;
	int 	err =	-ENODEV;

	ds3231 = devm_kzalloc( &client->dev , sizeof(struct ds3231) , GFP_KERNEL);
	if(!ds3231)
		return -ENOMEM;

	dev_set_dvrdata( &client->dev , ds3231 );
	ds3231->dev = &client->dev;
	ds3231->name = client->name;

	ds3231->regmap = devm_regmap_init_i2c( client , &regmap_config);
	if (IS_ERR(ds1307->regmap)) {
		dev_err(ds1307->dev, "regmap allocation failed\n");
		return PTR_ERR(ds1307->regmap);
	}	

	i2c_set_clientdata( client , ds3231);


}


struct i2c_driver ds3231_driver={
	.driver={
		.name	=	"ds3231",
	}:
	.probe		=	ds3231_probe,
		.id_table	=	ds3231_i2c_id,
};



module_i2c_driver(ds3231_driver);
MODULE_DESCRIPTION("RTC driver for DS3231");
MODULE_LICENSE("GPL");
