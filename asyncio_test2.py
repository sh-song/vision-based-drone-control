#!/usr/bin/env python3
import asyncio
async def run():
    print("This is the beginning of the script")
    print("Starting the secondary task...")
    secondary_task = asyncio.ensure_future(secondary_fun())
    await asyncio.sleep(1)
    print("Doing something in the run() function")
    await asyncio.sleep(1)
    print("Waiting for 'secondary_task' to finish...")
    await secondary_task
    print("secondary_fun() finished, ready to exit!")
async def secondary_fun():
    for i in range(0, 6):
        print(f"iteration {i}")
        await asyncio.sleep(0.5)
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())