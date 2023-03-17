for i in 1 2 3 4 5 6 7
do
   echo "inputs_chase_game/park$i.txt"
   ./../main_chase.o delta=4 dont_play play_safest play_controllable <../inputs_chase_game/park$i.txt
done
