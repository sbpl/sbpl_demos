# Instructions how to run sbpl_demos

On a local computer (which runs ROS Indigo), prepare three terminal tabs for the following machines:
* alan1 (c1 of PR2)
* alan2 (c2 of PR2)
* the local computer


## On alan1 tab

First of all, connect to alan1.
```
ssh demos@alan1
```

Then, open a new screen with the following commands.
```
roscd sbpl_demos
screen -c screen_config/screenrc_alan1
```

Each tab of screen should have the required command to run on alan1.

Check for additional comments on each screen tab, and run the commands in the order of screen tabs.


## On alan2 tab

Do the same things but for alan2.
```
ssh demos@alan2
roscd sbpl_demos
screen -c screen_config/screenrc_alan2
```


## On the local computer tab

Open a new screen with the following commands.
```
roscd sbpl_demos
screen -c screen_config/screenrc_tatooine
```

As a side note, set up the sytem time to California loal time (PST or PDT), just like PR2, to avoid any issues related to tf buffer.

You can also use `chrony` to syncing your computer to alanbase.
* Add `server alanbase minpoll 0 maxpoll 5 maxdelay 0.05` to your `/etc/chrony/chrony.conf`
* Restart `chrony` with `invoke-rc.d chrony restart`
* Check time offset with `ntpdate -q alanbase`

