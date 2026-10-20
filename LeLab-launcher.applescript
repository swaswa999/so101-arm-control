on run
    set launcherPath to (POSIX path of (path to desktop folder)) & "Projects/so101-arm-control/launch-lelab.sh"
    do shell script "/usr/bin/nohup /bin/zsh " & quoted form of launcherPath & " >/dev/null 2>&1 &"
end run
