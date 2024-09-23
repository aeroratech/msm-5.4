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
	struct drm_connector	connector;
	struct drm_bridge 	*next_bridge;
	struct platform_device	*device;
};

static inline struct failsafe_bridge *
drm_bridge_to_failsafe_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct failsafe_bridge, bridge);
}

static enum drm_connector_status fsbridge_detect(struct drm_connector *connector, bool force)
{
	return connector_status_disconnected;
}

static const struct drm_connector_funcs failsafe_bridge_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = fsbridge_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int fsbridge_get_modes(struct drm_connector *connector)
{
	return 0;
}

static const struct drm_connector_helper_funcs failsafe_bridge_connector_helper_funcs = {
	.get_modes = fsbridge_get_modes,
};

static int failsafe_bridge_attach(struct drm_bridge *bridge)
{
	struct failsafe_bridge *fsbridge = drm_bridge_to_failsafe_bridge(bridge);
	struct platform_device *platform_dev = fsbridge->device;
	struct device_node *remote;
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Missing encoder\n");
		return -ENODEV;
	}

	remote = of_graph_get_remote_node(platform_dev->dev.of_node, 0, -1);
	if (remote) {
		fsbridge->next_bridge = of_drm_find_bridge(remote);
		of_node_put(remote);

		if (fsbridge->next_bridge)
			return drm_bridge_attach(bridge->encoder, fsbridge->next_bridge, bridge);
		else {
			fsbridge->connector.polled = DRM_CONNECTOR_POLL_HPD;

			ret = drm_connector_init(bridge->dev, &fsbridge->connector,
					&failsafe_bridge_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
			if (ret < 0) {
				DRM_ERROR("Failed to initialize connector: %d", ret);
				return ret;
			}

			drm_connector_helper_add(&fsbridge->connector, &failsafe_bridge_connector_helper_funcs);
			drm_connector_attach_encoder(&fsbridge->connector, bridge->encoder);

			return 0;
		}
	}

	DRM_INFO("No bride found");

	return -ENODEV;
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

	if (!fsbridge->next_bridge) {
		drm_connector_unregister(&fsbridge->connector);
		drm_connector_cleanup(&fsbridge->connector);
	}
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
