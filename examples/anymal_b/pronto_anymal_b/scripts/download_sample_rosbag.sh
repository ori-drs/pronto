#!/bin/bash

wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1a_BA7yyj4XdUcCXrxpz5o1PdCi0fJn5K' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1a_BA7yyj4XdUcCXrxpz5o1PdCi0fJn5K" -O fsc_minimal_joint_states_short.zip && rm -rf /tmp/cookies.txt

unzip fsc_minimal_joint_states_short.zip
rm fsc_minimal_joint_states_short.zip

