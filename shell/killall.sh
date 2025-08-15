#!/bin/bash
echo "### kill all ros nodes and processes ###"
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
exit 0