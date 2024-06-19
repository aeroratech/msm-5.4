// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024, Aerora Technology, Inc.
 *
 * Authors:
 * James Zheng <james.zheng@aeroratech.com>
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

struct failsafe_bridge {
	struct drm_bridge	bridge;
	struct platform_device	*device;
};

static inline struct failsafe_bridge *
drm_bridge_to_failsafe_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct failsafe_bridge, bridge);
}

static int failsafe_bridge_attach(struct drm_bridge *bridge)
{
	struct failsafe_bridge *fsbridge = drm_bridge_to_failsafe_bridge(bridge);
	struct platform_device *platform_dev = fsbridge->device;
	struct drm_bridge *next_bridge;
	struct device_node *remote;

	if (!bridge->encoder) {
		DRM_ERROR("Missing encoder\n");
		return -ENODEV;
	}

	remote = of_graph_get_remote_node(platform_dev->dev.of_node, 0, -1);
	if (remote) {
		next_bridge = of_drm_find_bridge(remote);
		of_node_put(remote);

		if (next_bridge)
			return drm_bridge_attach(bridge->encoder, next_bridge, bridge);
	}

	DRM_INFO("No bride found");

	return 0;
}

static const struct drm_bridge_funcs failsafe_bridge_funcs = {
	.attach			= failsafe_bridge_attach,
};

static int failsafe_bridge_probe(struct platform_device *platform_dev)
{
	struct failsafe_bridge *fsbridge;

	fsbridge = devm_kzalloc(&platform_dev->dev, sizeof(*fsbridge), GFP_KERNEL);
	if (!fsbridge)
		return -ENOMEM;
	platform_set_drvdata(platform_dev, fsbridge);
	fsbridge->device = platform_dev;

	/* register the brdige. */
	fsbridge->bridge.funcs = &failsafe_bridge_funcs;
#ifdef CONFIG_OF
	fsbridge->bridge.of_node = platform_dev->dev.of_node;
#endif
	drm_bridge_add(&fsbridge->bridge);

	return 0;
}

static int failsafe_bridge_remove(struct platform_device *platform_dev)
{
	struct failsafe_bridge *fsbridge = platform_get_drvdata(platform_dev);

	drm_bridge_remove(&fsbridge->bridge);

	return 0;
}

static const struct of_device_id failsafe_bridge_of_match[] = {
	{ .compatible	= "failsafe-bridge" },
	{},
};
MODULE_DEVICE_TABLE(of, failsafe_bridge_of_match);

static struct platform_driver failsafe_bridge_driver = {
	.probe		= failsafe_bridge_probe,
	.remove		= failsafe_bridge_remove,
	.driver		= {
		.name		= "failsafe-drm-bridge",
		.of_match_table	= failsafe_bridge_of_match,
	},
};
module_platform_driver(failsafe_bridge_driver);

MODULE_AUTHOR("James Zheng <james.zheng@aeroratech.com>");
MODULE_DESCRIPTION("A failsafe DRM Bridge");
MODULE_LICENSE("GPL");
