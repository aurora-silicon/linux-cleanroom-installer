// SPDX-License-Identifier: GPL-2.0-only
/* Keep unsupported legacy display routes from blocking J700 USB probe. */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>

static int __init apple_j700_boot_handoff_init(void)
{
	struct device_node *dcp, *routes, *route;
	int ret = 0;

	if (!of_machine_is_compatible("apple,j700"))
		return 0;

	/*
	 * The retained standalone bootstrap describes an external-only DCP
	 * using the internal DCP compatible and legacy child route nodes.
	 * This kernel's T8140 DCP profile has no external transmitters and
	 * never registers those child muxes. Leave their endpoints unavailable
	 * to Type-C discovery instead of waiting for a provider that cannot
	 * exist. The standard PHY mode/orientation switches remain available.
	 * This is not needed by the native cleanroom J700 device tree.
	 */
	for_each_compatible_node(dcp, NULL, "apple,t8140-dcp") {
		if (!of_property_match_string(dcp, "apple,connector-type", "USB-C")) {
			routes = of_get_child_by_name(dcp, "typec-routes");
			for_each_available_child_of_node(routes, route) {
				struct of_changeset changes;

				if (!of_property_read_bool(route, "mode-switch"))
					continue;
				of_changeset_init(&changes);
				ret = of_changeset_update_prop_string(&changes, route,
								      "status", "disabled");
				if (!ret)
					ret = of_changeset_apply(&changes);
				of_changeset_destroy(&changes);
				if (ret)
					break;
				pr_info("apple-j700: disabled unsupported legacy DCP mux %pOF\n",
					route);
			}
			if (ret)
				of_node_put(route);
			of_node_put(routes);
		}
		if (ret) {
			of_node_put(dcp);
			break;
		}
	}
	return ret;
}
postcore_initcall(apple_j700_boot_handoff_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("J700 standalone USB and display boot handoff adapter");
