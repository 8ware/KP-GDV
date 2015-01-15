#! /usr/bin/env bash
#
# Call with 'bash testdata.sh 0 45 $(( $(ls -d ? | tail -n 1) + 1 ))
#
# [Kinect]
#  \  |  /
#   \ | /
#    \|/
#     o
#    /|\
#   / | \
#  /  |  \
#   [Arm]
#
# Arm is always 45cm away from center with angle 0°, while the position of the
# Kinect is specified by its angle and distance to the center.
#

if [ $# -lt 2 ]; then
	echo "usage: $(basename "$0") <angle> <distance>" >&2
	exit 1
fi

template="data_A$1-D$2"

# update build
make TestdataGenerator
generator="$PWD/TestdataGenerator"

#DONT_RECORD=true

#coords=(
#	"25 -35 10" "25 -35 20" "25 -35 30" "25 -35 40" "25 -35 50"
#	"25 0 50" "25 0 40" "25 0 30" "25 0 20" "25 0 10"
#	"25 35 10" "25 35 20" "25 35 30" "25 35 40" "25 35 50"
#	"-25 35 50" "-25 35 40" "-25 35 30" "-25 35 20" "-25 35 10"
#	"-25 0 10" "-25 0 20" "-25 0 30" "-25 0 40" "-25 0 50"
#	"-25 -35 50" "-25 -35 40" "-25 -35 30" "-25 -35 20" "-25 -35 10"
#)

function gen_coord() {
	coord=$(($RANDOM % ${2:-50}))
	sign=$( [ $(($RANDOM % ${1:-2})) == 0 ] && echo '' || echo '-' )
	echo "$(($coord * ${sign}1))"
}
coords=( 
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
	"$(gen_coord) $(gen_coord 2 40) $(gen_coord 1)${DONT_RECORD:+ DONT_RECORD}"
)

#[ $# -eq 3 ] && coords=( "9999 9999 9999" )

for xyz in "${coords[@]}"; do
	datadir="${template}_${xyz// /.}"
	if [ -d "$datadir" ]; then
		echo "Datadir $datadir already exists. Skipping..." >&2
		continue
	fi

	if [ "$DONT_RECORD" == true ]; then
		sudo "$generator" $xyz
		continue
	fi

	mkdir "$datadir"
	cd "$datadir"

	sudo "$generator" $xyz | egrep '.+ \| .+ : .+ \| .+' >> INFO
	sudo chown $USER:$USER *.png

	cd "$OLDPWD"
done

if [ $# -gt 2 ]; then
	mkdir $3
	mv data_* $3
fi

