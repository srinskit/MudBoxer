if test $# -lt 2; then
    echo 'Usage: ./makeManglerMap.sh some_ros_exe output_file'
    exit
fi
strings $1 | grep _ZN3ros > makeManglerMapTmp1
cat makeManglerMapTmp1 | c++filt > makeManglerMapTmp2
paste -d "'':''" - makeManglerMapTmp2 - - makeManglerMapTmp1 - < /dev/null | sort -u > $2
rm makeManglerMapTmp1 makeManglerMapTmp2