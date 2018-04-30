source ~/dotfiles/zshrc
cd build
cmake .. && make
DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/usr/local/lib
/Users/tylerfolkman/projects/carnd/CarND-Path-Planning-Project/build/path_planning
