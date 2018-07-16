#!/usr/bin/expect

set timeout 3
spawn ssh nrslnuc2@192.168.0.252
expect "*password*"
send "nrsl\r"
interact
