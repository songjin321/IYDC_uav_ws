#!/usr/bin/expect

set timeout 3
spawn ssh nrslnuc2@219.223.238.87
expect "*password*"
send "nrsl\r"
send "sudo -s\r"
interact