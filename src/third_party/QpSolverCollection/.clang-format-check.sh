#!/bin/bash

readonly this_dir=`cd $(dirname $0); pwd`
cd $this_dir
source .clang-format-common.sh

out=0
tmpfile=$(mktemp /tmp/clang-format-check.XXXXXX)
for f in ${src_files}; do
  $clang_format -style=file $f > $tmpfile
  if ! [[ -z `diff $tmpfile $f` ]]; then
    echo "Wrong formatting in $f"
    out=1
  fi
done
rm -f $tmpfile
if [[ $out -eq 1 ]]; then
  echo "You can run ./.clang-format-fix.sh to fix the issues locally, then commit/push again"
fi
exit $out
