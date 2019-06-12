#!/bin/bash
echo "Hello $i World"
ARRAY=(0 1)
for i in {1..5}
do
	# python2.7 test.py  $i "t"
	ARRAY[i]=$(python2.7 -c 'import test; print test.run()' ) # $i "t"
done
echo ${ARRAY[*]}
