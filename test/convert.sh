#! /usr/bin/env bash

source=${1:?Source directory must be given!}
target=${2:?Target directory must be given!}

for dir in "$source"/*; do
	angle=$(grep -oP 'data_A\K\d+' <<< "$dir")
	distance=$(grep -oP 'data_A\d+-D\K\d+' <<< "$dir")

	xyz=$(grep -oP 'data_A\d+-D\d+_\K.+' <<< "$dir")
	x=$(cut -d . -f 1 <<< "$xyz")
	y=$(cut -d . -f 2 <<< "$xyz")
	z=$(cut -d . -f 3 <<< "$xyz")

	xyz_=$(head -n 1 "$dir/INFO" | cut -d '|' -f 3 | grep -oP '(?<=\[).+(?=\])')
	x_=$(cut -d , -f 1 <<< "${xyz_// }")
	y_=$(cut -d , -f 2 <<< "${xyz_// }")
	z_=$(cut -d , -f 3 <<< "${xyz_// }")

	new="${x},${y},${z}_${x_},${y_},${z_}"
	echo "Converting $(basename "$dir") to $new (A=$angle, D=$distance)"

	mkdir -p "$target/$new"
	mv "$dir/"*.png "$target/$new"

	rm -v "$dir/INFO"
	rmdir -v "$dir"
done

echo "angle: ${angle}°" >> "$target/setup"
echo "distance: ${distance}cm" >> "$target/setup"

rmdir -v "$source"

