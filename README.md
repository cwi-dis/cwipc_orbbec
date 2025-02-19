# cwipc_orbbec

This project has software for capturing point clouds using Orbbec depth
cameras. The software turns Orbbec depth and color frames into a cwipc
pointcloud.

It is usually built or installed as part of the _cwipc_ suite, <https://github.com/cwi-dis/cwipc>.

This module requires the following software to be installed:

- Orbbec SDK
- OpenCV

See the [cwipc readme](../readme.md) file for installation instructions.

## Prepare for Use

The Orbbec capturer needs a `cameraconfig.json` file that specifies the serial
numbers of the cameras and their position and orientation.

If you have a single camera in landscape mode looking forward, approximately 1m
from the subject at 1m height you can automatically create a config file with

```
cwipc_register --tabletop
```

In the unlikely event that `cwipc_register` does not recognize the fact that
you have an Orbbec camera you can supply the `--orbbec` option.

If you have more cameras, or a single camera that is oriented differently (for
example in portrait mode, so it is easier to capture a full human body) you
should check the documentation on `cwipc_register`.

Inspect the resulting pointcloud view with

```
cwipc_view
```

## Configuration

You can edit `cameraconfig.json` and modify various settings, such as camera
parameters like white balance, various processing options that govern how RGB
and Depth images are converted to pointclouds, and bounding box parameters for
the pointcloud.

After editing parameters you re-run `cwipc_view` to see the effect of your
changes.
