#Timer 2 event
timer2.en=0
covx embedded.val,va2.txt,0,0
covx temp_number2.val,va1.txt,0,0
climatesetting.txt="{\"page\": \"climate\", \"key\": \"target_temp_low\", \"value\": "+va1.txt+", \"embedded\": "+va2.txt+"}"
printh 92
prints "localevent",0
printh 00
prints climatesetting.txt,0
printh 00
printh FF FF FF

/***********************************************************************************************/
### Slider_low
  #Touch Press Event
 temp_gap.val=3*temp_step.val
 temp_low_max.val=slider_high.val-temp_gap.val
 
/***********************************************************************************************/
  #Touch Release Event
 if(slider_low.val>temp_low_max.val)
  {
   slider_low.val=temp_low_max.val
  }
   temp_number2.val=slider_low.val*temp_step.val
 temp_number2.val+=temp_offset.val
 va0.val=temp_number2.val/10
 covx va0.val,target_low.txt,0,0
 va0.val=temp_number2.val%10
 covx va0.val,va1.txt,0,0
 target_low.txt+=dec_separator.txt+va1.txt
 target_low.txt+=temp_unit.txt
 timer2.en=1
 active_slider.val=2
 temp_gap.val=3*temp_step.val
 temp_high_min.val=slider_low.val+temp_gap.val

/***********************************************************************************************/
  #Touch Move
 if(slider_low.val>temp_low_max.val)
  {
   slider_low.val=temp_low_max.val
  }
 temp_number2.val=slider_low.val*temp_step.val
 temp_number2.val+=temp_offset.val
 va0.val=temp_number2.val/10
 covx va0.val,target_low.txt,0,0
 va0.val=temp_number2.val%10
 covx va0.val,va1.txt,0,0
 target_low.txt+=dec_separator.txt+va1.txt
 target_low.txt+=temp_unit.txt
