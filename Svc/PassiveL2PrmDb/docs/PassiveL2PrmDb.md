<title>PassiveL2PrmDb Component Dictionary</title>
# PassiveL2PrmDb Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|L2_PRM_PUSH_UPDATES|1 (0x1)|Push parameter updates to the L1 Prm Db| | |   
| | | |updateMethod|UpdateMethod||                    


## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|PrmIdNotFound|0 (0x0)|Parameter ID not found in database.| | | | |
| | | |Id|U32||The parameter ID|    
|PrmIdUpdated|1 (0x1)|Parameter ID updated in database| | | | |
| | | |Id|U32||The parameter ID|    
|PrmDbFull|2 (0x2)|Parameter database is full| | | | |
| | | |Id|U32||The parameter ID|    
|PrmIdAdded|3 (0x3)|Parameter ID added to database| | | | |
| | | |Id|U32||The parameter ID|    
|PrmSendTooLarge|8 (0x8)|Parameter too large to send to level 1 PrmDb| | | | |
| | | |prmId|U32||Parameter that could not be sent|    
| | | |prmSize|U32||Size of serialized parameter|    
|PrmSendBufferFull|9 (0x9)|| | | | |
