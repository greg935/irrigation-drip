# irrigation-drip

Irrigation drip is a small project to create an irrigation controller for an orchard.

It includes schedules and backwash cycle.

It is based on an ESP32 board and programmed via Arduino IDE with ESP32 extensions.

It was adapted from an earlier version running on an Arduino Uno.

This differs from most irrigation controllers in that it is more about scale of the irrigation than time. 
So it doesn't have a weekly schedule and is designed as a setup and run once.
Normal farmers don't go away for a week or two like home gardeners and so each irrigation cycle is run separately and as needed based on weather and soil moisture.
This controller helps to avoid the wake up at 3am to change the pump to the next section of orchard problem.
