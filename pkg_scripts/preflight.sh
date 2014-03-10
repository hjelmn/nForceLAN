#!/bin/sh

[ -d /System/Library/Extensions.disabled ] || mkdir /System/Library/Extensions.disabled
[ -d /System/Library/Extensions.disabled/nForce ] || mkdir /System/Library/Extensions.disabled/nForce

# move all conflicting extensions out of the way
move=( nvenet.kext nForceEthernetController.kext nForceLAN.kext forcedeth.kext forcedeth-d.kext forcedeth-nock.kext forcedeth-nockd.kext )

for i in 0 1 2 3 4 5 6; do
    [ -d /System/Library/Extensions/${move[i]} ] && [ -d /System/Library/Extensions.disabled/nForce/${move[i]} ] && rm -rf /System/Library/Extensions.disabled/nForce/${move[i]}
    [ -d /System/Library/Extensions/${move[i]} ] && mv -f /System/Library/Extensions/${move[i]} /System/Library/Extensions.disabled/nForce/
done

if [ -f /System/Library/Extensions/IONetworkingFamily.kext/Contents/Plugins/nvenet.kext ]; then
	mv /System/Library/Extensions/IONetworkingFamily.kext/Contents/Plugins/nvenet.kext /System/Library/Extensions.disabled/nForce/nvenet_IONetworkingFamily.kext
fi

exit 0
