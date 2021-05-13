import asyncio

async def add(a, b):
    print('add: {0} + {1}'.format(a,b))
    print('Waiting for 2 secs')
    await asyncio.sleep(2.0)
    return a + b

async def print_add(a, b):
    result = await add(a, b)
    print('===============')
    print('Waited for 2 secs')
    print('print_add: {0} + {1} = {2}'.format(a, b, result))
    
loop = asyncio.get_event_loop()
print('Loop Start')
loop.run_until_complete(print_add(1, 2))
print('Loop Done')
loop.close