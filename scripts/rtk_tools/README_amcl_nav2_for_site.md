# AMCL Nav2 Independent Workflow

This workflow is separate from the RTK Nav2 workflow.

RTK Nav2 remains:

```bash
/home/wheeltec/rtk_tools/start_nav2_for_site.sh --site 整体户外测试4
```

AMCL Nav2 is:

```bash
/home/wheeltec/rtk_tools/start_amcl_nav2_for_site.sh --site 整体户外测试4
```

AMCL mode does not start RTK, NTRIP, `publish_rtk_map_tf.py`, or RTK map anchoring.
It uses AMCL to publish `map -> odom_combined`, so the robot must be initialized in RViz with `2D Pose Estimate`.

The semantic place and route scripts can still be used after AMCL localization is correct:

```bash
/home/wheeltec/rtk_tools/navigate_to_place.py --site 整体户外测试4 --place C
/home/wheeltec/rtk_tools/execute_nav2_route.py --site 整体户外测试4 --route outdoor_test4_route_1
```

Do not run RTK Nav2 and AMCL Nav2 at the same time.
