# Adjust these paths as needed; PKG_CONFIG_PATH lets pkg-config find PCL/OMP/FLANN.
export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:/opt/homebrew/opt/llvm/lib/pkgconfig:/opt/homebrew/Cellar/pcl/1.15.1_1/lib/pkgconfig:/opt/homebrew/opt/flann/lib/pkgconfig:${PKG_CONFIG_PATH}"

# Fallbacks if pkg-config is missing/incomplete
export PCL_INCLUDE_DIR=${PCL_INCLUDE_DIR:-/opt/homebrew/include/pcl-1.15}
export PCL_LIB_DIR=${PCL_LIB_DIR:-/opt/homebrew/lib}
export EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR:-/opt/homebrew/include/eigen3}
export OMP_LIB_DIR=${OMP_LIB_DIR:-/opt/homebrew/opt/llvm/lib}
export OMP_INCLUDE_DIR=${OMP_INCLUDE_DIR:-/opt/homebrew/opt/llvm/include}

cargo clean && cargo build
