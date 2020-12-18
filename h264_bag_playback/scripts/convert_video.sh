
input_folder=$1

find $input_folder/* -prune -type d | while IFS= read -r d; do
  for input_file in $d/*[0-9].h264; do
    if [ ! -f "$input_file" ]; then
      echo "Could not find video files in $d"
    else
      output_file=$(echo $input_file | sed "s/.h264/.mp4/g")
      temp_file=$(echo $input_file | sed "s/.h264/.tmp.mp4/g")
      echo "Converting $input_file to $output_file"
      if [ ! -f "$output_file" ]; then
        echo "Will convert $input_file"
        ffmpeg -y -i $input_file -map 0:v -vcodec copy -bsf:v h264_mp4toannexb $temp_file
        ffmpeg -y -fflags +genpts -r 30 -i $temp_file -vcodec copy $output_file
        rm $temp_file
      else
        echo "$output_file already exists"
      fi
    fi
  done
done

