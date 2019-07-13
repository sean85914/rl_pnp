BEGIN=1
END=100
for ((i=$BEGIN; i<=$END; i++))
do
  rosservice call /get_reward/set_prior
  rosservice call /get_reward/set_posterior
  rosservice call /get_reward/get_result "data: false"
  echo "$i"
done
