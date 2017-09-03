#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <sound/soc.h>
#include <sound/jack.h>

extern struct snd_soc_card byt_rt5640_card;

static const struct acpi_gpio_params soc_gpios = { 1, 0, false };

static const struct acpi_gpio_mapping acpi_byt_rt5640_mc_gpios[] = {
	{ "soc-gpios", &soc_gpios, 1 },
	{ },
};

static struct snd_soc_jack_pin byt_rt5640_mc_jack_pin = {
	.pin	= "Headphone",
	.mask	= SND_JACK_HEADPHONE,
};

static struct snd_soc_jack_gpio byt_rt5640_mc_jack_gpio = {
	.name = "soc",
	.report = SND_JACK_HEADPHONE,
	.invert = 1,
	.debounce_time = 500,
};

static int snd_byt_rt5640_mc_jack_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_jack *jack;
	struct snd_soc_card *card = &byt_rt5640_card;

	if (!card->instantiated) {
		return -EPROBE_DEFER;
	}

	ret = acpi_dev_add_driver_gpios(ACPI_COMPANION(&pdev->dev),
			acpi_byt_rt5640_mc_gpios);
	if (ret) {
		return ret;
	}

	jack = devm_kzalloc(&pdev->dev, sizeof(*jack), GFP_KERNEL);
	if (!jack) {
		return -ENOMEM;
	}

	ret = snd_soc_card_jack_new(card, "Headphone", SND_JACK_HEADPHONE, jack,
					&byt_rt5640_mc_jack_pin, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register audio jack: %d\n", ret);
		return ret;
	}

	/* Register the new audio jack */
	ret = snd_device_register(card->snd_card, jack->jack);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register audio jack: %d\n", ret);
		return ret;
	}

	ret = snd_soc_jack_add_gpiods(&pdev->dev, jack, 1, &byt_rt5640_mc_jack_gpio);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add audio jack GPIOs: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, jack);
	return 0;
}

static int snd_byt_rt5640_mc_jack_remove(struct platform_device *pdev)
{
	struct snd_soc_jack *jack = platform_get_drvdata(pdev);

	snd_soc_jack_free_gpios(jack, 1, &byt_rt5640_mc_jack_gpio);
	acpi_dev_remove_driver_gpios(ACPI_COMPANION(&pdev->dev));
	return 0;
}

static const struct acpi_device_id snd_byt_rt5640_mc_jack_acpi_ids[] = {
	{ "AMCR0F28", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, snd_byt_rt5640_mc_jack_acpi_ids);

static struct platform_driver snd_byt_rt5640_mc_jack_driver = {
	.driver = {
		.name = "bytcr_rt5640_jack",
		.acpi_match_table = ACPI_PTR(snd_byt_rt5640_mc_jack_acpi_ids),
	},
	.probe = snd_byt_rt5640_mc_jack_probe,
	.remove = snd_byt_rt5640_mc_jack_remove,
};

module_platform_driver(snd_byt_rt5640_mc_jack_driver);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail CR audio jack driver");
MODULE_AUTHOR("Lambdadroid <lambdadroid@gmail.com>");
MODULE_LICENSE("GPL v2");
