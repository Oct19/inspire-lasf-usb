# Inspire Robots 因时电机 LASF16 USB

Inspire Robot（因时机器人 / 因时电机）LASF16 linear servo on USB. One CH340 board, COM port, 921600 8N1, two actuator ids. Optional NDI Polaris Vega on Ethernet. The page is http://127.0.0.1:8766/ .

Search terms: Inspire, Inspire Robots, 因时, 因时电机, 因时机器人, LASF16, USB, CH340.

This repository is only the motor and tracker page. It is a normal Git repository, not a submodule.

```
py -3.13 -m pip install -r requirements.txt
py -3.13 python/bench_ui.py
```

Close the 因时 vendor host program first. It holds the COM port. Do not flash parameters, change id or baud, or open the tracker on the motor COM port.

Logs go to `logs/` and are not committed.

Control notes: [NOTES.md](NOTES.md).

## Upload

Create an empty GitHub repository named `inspire-lasf-usb`, with no README, then:

```
git init
git add .
git commit -m "Start the Inspire LASF16 USB page and tracker note."
git branch -M main
git remote add origin https://github.com/YOU/inspire-lasf-usb.git
git push -u origin main
```

Set the GitHub description to: Inspire Robots 因时电机 LASF16 USB control page, with NDI Polaris Vega.

Topics that help search: `inspire`, `inspire-robots`, `lasf16`, `usb-serial`. The Chinese name is in this README, which GitHub search indexes.
