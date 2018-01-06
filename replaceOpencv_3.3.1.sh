FILES="/usr/local/lib/libopencv*"
pathCopy="/opt/ros/kinetic/lib/"
pattherDelete=$pathCopy"libopencv*"


for f in $pattherDelete
do
	sudo rm $f
done

for f in $FILES
do
	filename=$(basename "$f")
	extension="${filename##*.}"
	if [ $extension = "so" ]; then
		sudo cp $f $pathCopy
		newfilename="${filename%.*}"3.$extension
		sudo mv $pathCopy$filename $pathCopy$newfilename
	#	sudo echo $newfilename
		cd $pathCopy
		sudo ln -s $newfilename.3.3 $newfilename.3.3.1
		sudo ln -s $newfilename $newfilename.3.3
	fi
	#echo $f
	#mv $f $f
done
