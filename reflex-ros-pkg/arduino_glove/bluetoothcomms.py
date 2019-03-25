import bluetooth
print "performing inquiry...."
nearby_devices = bluetooth.discover_devices(duration=10,lookup_names=True)

print "found %d devices" % len(nearby_devices)

for name, addr in nearby_devices:
     print " %s - %s" % (addr, name)
