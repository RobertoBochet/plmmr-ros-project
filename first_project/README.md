# First ROS project

## Packages

### `raw_data_handler`

Its job is convert raw gps data (`NavSatFix`) to tf and odometric data (`Odometry`).
The data are also converted to ENU convection.

#### Out of sequence messages

Since the gps messages can arrive out of sequence a `TimeSequencer` filter is used to rearrange these.

#### gps signal loss

If gps signal is lost the odometry package is flagged by the `child_frame_id` set as `{name}_lost` (instead of `{name}), and tf is not published.

#### Parameters

##### globally

- `initial/latitude` initial latitude
- `initial/longitude` initial longitude
- `initial/altitude` initial altitude
- `debug` [optional] if true odometry x/100 is published on `/odom/debug/{name}`

##### private

- `topic` gps data raw topic
- `name` name of the node, it is used for publishing topic and `child_frame_id`
- `reference` reference frame id for tf and odometry


### `status_check`

This package provides a node subscribes to two odometric topics with `ApproximateTime` sync policy. When two messages arrive it checks if both are valid, it calculates the distance exploiting the `distance_calculator` service. In addition, the node determines collision status, then it publishes the values on the topic `/status`.  
The two threshold can be dynamically changed.


#### Parameters

##### private

- `safe_limit` minimum distance to flag status as `safe`
- `crash_limit` minimum distance to flag status as `unsafe`


### `distance_service`

This package provides a service takes two 3d point (`Point`) and return the distance (`float64`) between them.


### `orchestrator`

The package provides only the launch file. 


## TF

The tf root is `map` and `front` and `obs` are its children.


## Custom messages

### Status

The message is used in topic `/status` and provides `distance` (`float64`) and `status` (`string`).
`status` can take the following values `safe`, `unsafe`, `crash` and `unavailable`.


## How to start

It is sufficient to use the launch file in the package `orchestrator` called `orchestrator.launch`.
