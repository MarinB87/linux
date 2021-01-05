#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#define PIANO_PWM_MAX_DUTY			255
#define PIANO_PWM_PERIOD		    10000
#define PIANO_NUM_PWMS			    64
#define PIANO_PWM_MAX_REGISTERS		0x3F

enum piano_pwm_output {
	PIANO_PWM_OUT0, PIANO_PWM_OUT1, PIANO_PWM_OUT2, PIANO_PWM_OUT3, PIANO_PWM_OUT4, PIANO_PWM_OUT5, PIANO_PWM_OUT6, PIANO_PWM_OUT7,
	PIANO_PWM_OUT8, PIANO_PWM_OUT9, PIANO_PWM_OUT10, PIANO_PWM_OUT11, PIANO_PWM_OUT12, PIANO_PWM_OUT13, PIANO_PWM_OUT14, PIANO_PWM_OUT15,
	PIANO_PWM_OUT16, PIANO_PWM_OUT17, PIANO_PWM_OUT18, PIANO_PWM_OUT19, PIANO_PWM_OUT20, PIANO_PWM_OUT21, PIANO_PWM_OUT22, PIANO_PWM_OUT23,
	PIANO_PWM_OUT24, PIANO_PWM_OUT25, PIANO_PWM_OUT26, PIANO_PWM_OUT27, PIANO_PWM_OUT28, PIANO_PWM_OUT29, PIANO_PWM_OUT30, PIANO_PWM_OUT31,
	PIANO_PWM_OUT32, PIANO_PWM_OUT33, PIANO_PWM_OUT34, PIANO_PWM_OUT35, PIANO_PWM_OUT36, PIANO_PWM_OUT37, PIANO_PWM_OUT38, PIANO_PWM_OUT39,
	PIANO_PWM_OUT40, PIANO_PWM_OUT41, PIANO_PWM_OUT42, PIANO_PWM_OUT43, PIANO_PWM_OUT44, PIANO_PWM_OUT45, PIANO_PWM_OUT46, PIANO_PWM_OUT47,
	PIANO_PWM_OUT48, PIANO_PWM_OUT49, PIANO_PWM_OUT50, PIANO_PWM_OUT51, PIANO_PWM_OUT52, PIANO_PWM_OUT53, PIANO_PWM_OUT54, PIANO_PWM_OUT55,
	PIANO_PWM_OUT56, PIANO_PWM_OUT57, PIANO_PWM_OUT58, PIANO_PWM_OUT59, PIANO_PWM_OUT60, PIANO_PWM_OUT61, PIANO_PWM_OUT62, PIANO_PWM_OUT63,
};

/*
 * struct piano_pwm_map
 * @output: Output pin which is mapped to each PWM channel. We have one pin per channel
 */
struct piano_pwm_map {
	enum piano_pwm_output output;
};

struct piano_fpga_pwm {
	struct pwm_chip chip;
	struct regmap *regmap;
	struct piano_pwm_map *pwms[PIANO_NUM_PWMS];
	unsigned long pin_used[2];
};

static inline struct piano_fpga_pwm *to_piano_fpga_pwm(struct pwm_chip *_chip)
{
	return container_of(_chip, struct piano_fpga_pwm, chip);
}

static struct piano_pwm_map *
piano_pwm_request_map(struct piano_fpga_pwm *piano_fpga_pwm, int hwpwm)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_request_map\n");

	struct piano_pwm_map *pwm_map;
	int offset;

	pwm_map = kzalloc(sizeof(*pwm_map), GFP_KERNEL);
	if (!pwm_map)
		return ERR_PTR(-ENOMEM);

	pwm_map->output = piano_fpga_pwm->pwms[hwpwm]->output;
	offset = pwm_map->output;

	if (offset < (sizeof(unsigned long) * 8)) {
	    /* Return an error if the pin is already assigned */
	    if (test_and_set_bit(offset, &piano_fpga_pwm->pin_used[0])) {
		    kfree(pwm_map);
		    return ERR_PTR(-EBUSY);
	    }			
	} else {
		/* Return an error if the pin is already assigned */
	    if (test_and_set_bit(offset - (sizeof(unsigned long) * 8), &piano_fpga_pwm->pin_used[1])) {
		    kfree(pwm_map);
		    return ERR_PTR(-EBUSY);
	    }
	}

	return pwm_map;
}

static int piano_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_request\n");

	struct piano_fpga_pwm *piano_fpga_pwm = to_piano_fpga_pwm(chip);
	struct piano_pwm_map *pwm_map;

	pwm_map = piano_pwm_request_map(piano_fpga_pwm, pwm->hwpwm);
	if (IS_ERR(pwm_map))
		return PTR_ERR(pwm_map);

	return pwm_set_chip_data(pwm, pwm_map);
}

static void piano_pwm_free_map(struct piano_fpga_pwm *piano_fpga_pwm,
				struct piano_pwm_map *pwm_map)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_free_map\n");

	int offset;

	offset = pwm_map->output;

	if (offset < (sizeof(unsigned long) * 8)) {
	    clear_bit(offset, &piano_fpga_pwm->pin_used[0]);	
	} else {
		clear_bit(offset - (sizeof(unsigned long) * 8), &piano_fpga_pwm->pin_used[1]);
	}

	kfree(pwm_map);
}

static void piano_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_free\n");

	struct piano_fpga_pwm *piano_fpga_pwm = to_piano_fpga_pwm(chip);
	struct piano_pwm_map *pwm_map = pwm_get_chip_data(pwm);

	piano_pwm_free_map(piano_fpga_pwm, pwm_map);
}

static int piano_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_config\n");

	struct piano_fpga_pwm *piano_fpga_pwm = to_piano_fpga_pwm(chip);
	u8 val, reg_duty;

	/* There is only one register for each PWM of piano FPGA, and this is the duty cycle.
	*  Period is fixed to a value of 10000 ns so we can only change the duty cycle.
	*  Registers for configuring the duty cycle are adjacent, starting from address 0 */

	reg_duty = (u8)(pwm->hwpwm);
	val = (u8)(duty_ns * PIANO_PWM_MAX_DUTY / PIANO_PWM_PERIOD);

	/* Do not write the duty cycle to register if PWM is disabled */

	if(pwm->state.enabled)
		regmap_write(piano_fpga_pwm->regmap, reg_duty, val);

	return 0;
}

static int piano_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_enable\n");

	struct piano_fpga_pwm *piano_fpga_pwm = to_piano_fpga_pwm(chip);
	u8 val, reg_duty;

	reg_duty = (u8)(pwm->hwpwm);
	val = (u8)(pwm->state.duty_cycle * PIANO_PWM_MAX_DUTY / PIANO_PWM_PERIOD);
	/*
		* Setting the Piano PWM duty cycle also enables it, since writing
		* a value greater than 0 to duty cycle register enables the PWM output 
	 */

	return regmap_write(piano_fpga_pwm->regmap, reg_duty, val);
}

static void piano_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_disable\n");

	struct piano_fpga_pwm *piano_fpga_pwm = to_piano_fpga_pwm(chip);
	u8 val, reg_duty;

	/*
	 * Simply write 0 to duty cycle register to disable the PWM
	 */

	reg_duty = (u8)(pwm->hwpwm);
	val = 0;

	regmap_write(piano_fpga_pwm->regmap, reg_duty, val);
}

static const struct pwm_ops piano_pwm_ops = {
	.request	= piano_pwm_request,
	.free		= piano_pwm_free,
	.config		= piano_pwm_config,
	.enable		= piano_pwm_enable,
	.disable	= piano_pwm_disable,
	.owner		= THIS_MODULE,
};

static int piano_pwm_set_outputs(struct device *dev,
			       struct piano_fpga_pwm *piano_fpga_pwm)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_parse_dt\n");

	struct piano_pwm_map *pwm_map;
	enum piano_pwm_output *output;
	int i, count = 0;

	for (i = 0; i < PIANO_NUM_PWMS; i++) {
		output = devm_kcalloc(dev, 1, sizeof(*output),
				      GFP_KERNEL);
		if (!output)
			return -ENOMEM;

		*output = i;

		pwm_map = devm_kzalloc(dev, sizeof(*pwm_map), GFP_KERNEL);
		if (!pwm_map)
			return -ENOMEM;

		pwm_map->output = *output;
		piano_fpga_pwm->pwms[i] = pwm_map;

		count++;
	}

	if (count == 0)
		return -ENODATA;

	return 0;
}

static const struct regmap_config piano_fpga_pwm_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PIANO_PWM_MAX_REGISTERS,
};

static int piano_pwm_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_probe\n");
	struct piano_fpga_pwm *piano_fpga_pwm;
	struct device *dev = &cl->dev;
	int ret;

	piano_fpga_pwm = devm_kzalloc(dev, sizeof(*piano_fpga_pwm), GFP_KERNEL);
	if (!piano_fpga_pwm)
		return -ENOMEM;

	piano_fpga_pwm->regmap = devm_regmap_init_i2c(cl, &piano_fpga_pwm_regmap_config);
	if (IS_ERR(piano_fpga_pwm->regmap))
		return PTR_ERR(piano_fpga_pwm->regmap);

	i2c_set_clientdata(cl, piano_fpga_pwm);

	if (IS_ENABLED(CONFIG_OF)) {
		printk(KERN_ALERT "PIANO-PWM - calling piano_pwm_parse_dt\n");
		ret = piano_pwm_set_outputs(dev, piano_fpga_pwm);
	} else {
		printk(KERN_ALERT "PIANO-PWM - ENODEV\n");
		ret = -ENODEV;
	}

	if (ret) {
		printk(KERN_ALERT "PIANO-PWM - return ret\n");
		return ret;
	}

	piano_fpga_pwm->chip.dev = dev;
	piano_fpga_pwm->chip.ops = &piano_pwm_ops;
	piano_fpga_pwm->chip.base = -1;
	piano_fpga_pwm->chip.npwm = PIANO_NUM_PWMS;

	printk(KERN_ALERT "PIANO-PWM - calling pwmchip_add\n");
	return pwmchip_add(&piano_fpga_pwm->chip);
}

static int piano_pwm_remove(struct i2c_client *client)
{
	printk(KERN_ALERT "PIANO-PWM - piano_pwm_remove\n");

	struct piano_fpga_pwm *piano_fpga_pwm = i2c_get_clientdata(client);

	return pwmchip_remove(&piano_fpga_pwm->chip);
}

#ifdef CONFIG_OF
static const struct of_device_id piano_pwm_of_match[] = {
	{ .compatible = "mb,piano-pwm", },
	{ }
};
MODULE_DEVICE_TABLE(of, piano_pwm_of_match);
#endif

static struct i2c_driver piano_pwm_driver = {
	.probe = piano_pwm_probe,
	.remove = piano_pwm_remove,
	.driver = {
		.name = "piano-pwm",
		.of_match_table = of_match_ptr(piano_pwm_of_match),
	},
};
module_i2c_driver(piano_pwm_driver);

MODULE_DESCRIPTION("Piano PWM driver");
MODULE_ALIAS("platform:piano-pwm");
MODULE_AUTHOR("Marin Basic");
MODULE_LICENSE("GPL");
