sed "s/robots_[0-9]*.inc/robots_${1}.inc/g" multi.world > sed.tmp
cat sed.tmp > multi.world
rm -f sed.tmp
