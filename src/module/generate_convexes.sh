#!/usr/bin/bash

# This script is used to generate the convex files from the meshes in ../meshes

# Requires
# - qconvex (sudo apt install qhull-bin)
# - mesh_sampling (https://github.com/jrl-umi3218/mesh_sampling)

readonly this_dir=`cd $(dirname $0); pwd`
readonly meshes_dir=`cd $this_dir/meshes; pwd`

echo "Meshes dir is $meshes_dir"
for dir in `find $meshes_dir -mindepth 1 -type d`
do
  echo "Processing $dir"
  dir_name=`basename $dir`
  echo "dir_name=$dir_name"
  tmp_dir="/tmp/panda_prosthesis_convex/$dir_name"
  mkdir -p $tmp_dir
  echo "tmp_dir=$tmp_dir"
  for m in `find $dir -type f -name '*.stl'`
  do
    qc_out="$tmp_dir/`basename $m '.stl'`.qc"
    echo "qc_out=$qc_out"
    ch_out=$this_dir/convex/$dir_name/`basename $m ".stl"`-ch.txt
    echo "ch_out=$ch_out"
    mesh_sampling $m $qc_out --type xyz --samples 1000 --scale 0.001
    qconvex TI $qc_out TO $ch_out Qt o f
    # rm -f $qc_out
  done
done
