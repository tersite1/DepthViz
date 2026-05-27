# Third-Party Dependencies

## Eigen (Included)
Eigen 3.4.0 is included in the `Eigen` directory. It is a header-only library.

## PCL (Point Cloud Library) - REQUIRED
Fast-LIO2 depends on PCL for point cloud data structures (`pcl::PointCloud`, `pcl::PointXYZINormal`, etc.).
Since PCL is large and requires compilation for iOS (arm64), it is NOT included here.

### How to Install PCL for iOS
1. You need to build PCL as a static library (`.a`) or framework for iOS.
2. Recommended tool: `pcl-for-ios` (search on GitHub).
3. Once built, place the headers in `pcl/include` and the library in `pcl/lib`.
4. Add the library to your Xcode project's "Link Binary with Libraries" phase.
5. Add the header search paths to your Build Settings.

**Note:** If you want to run Fast-LIO2 without full PCL, you would need to rewrite `ikd-Tree` and `common_lib.h` to use a custom PointCloud struct instead of `pcl::PointCloud`. This is a significant effort but removes the heavy dependency.
