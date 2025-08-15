#!/bin/bash

readonly this_dir=`cd $(dirname $0); pwd`
cd $this_dir
source .clang-format-common.sh

for f in ${src_files}; do
  $clang_format -style=file -i $f
done
