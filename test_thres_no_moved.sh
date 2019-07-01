BEGIN=1
END=100
for ((i=$BEGIN; i<=$END; i++))
do
  rosservice call /get_action_state/set_prior
  rosservice call /get_action_state/set_posterior
  rosservice call /get_action_state/get_result
  echo "$i"
done
