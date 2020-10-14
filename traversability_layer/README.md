# Traversability Layer
`costmap_2d` plugin that transforms an input traversability map (message type `grid_map_msgs`) into a costmap for `move_base`.

## Parameters
* **ns** (string): namespace of the layer
* **topic** (string): traversability map topic
* **no_readings_timeout** (double): rate at which we expect input messages
* **traversability_layer** (string): layer name containing traversability information in the input map
* **traversable_threshold** (double): value in the interval [0,1] such that a position is considered traversable
* **enabled** (bool): set to `True` to enable this layer (it can be modified via `rqt_dynamic_reconfigure`)
* **use_maximum** (bool): set to true to threshold all the costs to a max value
