# Troubleshooting

## Asyncio Warning

While running RoSys you may see warnings similar to this one:

```
2021-10-31 15:08:04.040 [WARNING] asyncio: Executing <Task pending name='Task-255' coro=<handle_event() running at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:344> wait_for=<_GatheringFuture pending cb=[<TaskWakeupMethWrapper object at 0x7f7001f8e0>()] created at /usr/local/lib/python3.9/asyncio/tasks.py:705> created at /usr/local/lib/python3.9/site-packages/justpy/justpy.py:261> took 0.238 seconds
```

This means some coroutine is clogging the event loop for too long.
In the above example it is a whopping 238 ms in which no other actor can do anything.
This is an eternity when machine communication is expected to happen about every 10 ms.
The warning also provides a (not so readable) hint where the time is consumed.

The example above is one of the more frequent scenarios.
It means some code inside a user interaction event handler (e.g. `handle_event()` in `justpy.py`) is blocking.
Try to figure out which UI event code is responsible by commenting out parts of your logic and try to reproduce the warning systematically.

## CairoSVG on Mac

If [CairoSVG](https://cairosvg.org/) was installed via [Homebrew](https://brew.sh/), python sometimes can't find the correct path to run CairoSVG.
This will create a symbolic link to make the library accessible.

```bash
sudo mkdir -p /usr/local/lib && sudo ln -sf /opt/homebrew/lib/libcairo.2.dylib /usr/local/lib/libcairo.2.dylib
```

You can test it with this command:

```python
python3 -c "import cairocffi; import cairosvg; print('Cairo packages successfully imported!')"
```
