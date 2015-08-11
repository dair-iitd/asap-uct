width=$1
end=$3
diff=$4
while [ $width -lt $end ]
do
   ./makeSAU.sh 10 $width $2 >> answer_l$2 
   width=`expr $width + $diff`
done
