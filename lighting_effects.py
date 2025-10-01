import math



async def bounce_stairs(delay, intensities):
    n_steps = len(intensities)
    for step_n in range(n_steps):
        intensities[step_n] =255
        await asyncio.sleep(delay)
