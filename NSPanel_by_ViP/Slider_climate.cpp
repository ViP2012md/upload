### Timer 0 event
timer0.en=0
covx embedded.val,va2.txt,0,0
covx temp_number0.val,va1.txt,0,0
climatesetting.txt="{\"page\": \"climate\", \"key\": \"temperature\", \"value\": "+va1.txt+", \"embedded\": "+va2.txt+"}"
printh 92
prints "localevent",0
printh 00
prints climatesetting.txt,0
printh 00
printh FF FF FF

/***********************************************************************************************/
### Climateslider
  # Touch Press Event
 //Empty
 //Empty
 
/***********************************************************************************************/
  #Touch Release Event
 
 
 
 
 temp_number0.val=climateslider.val*temp_step.val
 temp_number0.val+=temp_offset.val
 va0.val=temp_number0.val/10
 covx va0.val,target_high.txt,0,0
 va0.val=temp_number0.val%10
 covx va0.val,va1.txt,0,0
 target_high.txt+=dec_separator.txt+va1.txt
 target_high.txt+=temp_unit.txt
 timer0.en=1
 active_slider.val=0
 
 
 
 /***********************************************************************************************
  # Touch Move




 temp_number0.val=climateslider.val*temp_step.val
 temp_number0.val+=temp_offset.val
 va0.val=temp_number0.val/10
 covx va0.val,target_high.txt,0,0
 va0.val=temp_number0.val%10
 covx va0.val,va1.txt,0,0
 target_high.txt+=dec_separator.txt+va1.txt
 target_high.txt+=temp_unit.txt
