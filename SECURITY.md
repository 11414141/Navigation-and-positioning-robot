# Security and Privacy

Do not commit secrets or site data to this public repository. This includes NTRIP/caster credentials, access tokens, private keys, Wi-Fi credentials, public IP addresses, robot serial numbers, exact field coordinates, rosbag files, maps, semantic waypoint files, or raw runtime logs.

The RTK helper scripts require a local private launch file. Keep it outside this repository and provide its path through `GNSS_PRIVATE_LAUNCH`. Example:

```bash
export GNSS_PRIVATE_LAUNCH=/absolute/path/to/zed_f9p_ntrip_private.launch.py
```

Before pushing changes, run:

```bash
rg -n -i 'password|passwd|secret|token|api[_-]?key|private[ _-]?key|username|credential|ntrip.*(user|pass|host)' .
find . -type f -size +90M -print
git status --short
```

If a secret was committed, revoke or rotate it immediately. Removing it only in a later commit does not remove it from Git history.
