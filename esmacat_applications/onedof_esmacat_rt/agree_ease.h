#ifndef AGREE_EASE_H
#define AGREE_EASE_H

#include "ethercat_arduino_shield_by_esmacat.h"


class agree_ease : public esmacat_ethercat_arduino_shield_by_esmacat
{
private:
   int conversion_factor = 50;

   bool   robot_referenced = false;
   bool   calibration_ready = false;
   int32_t calibration_reference_counts[5];

   int calibration_ready_index = 7;
   int robot_referenced_index = 6;

public:
    agree_ease();

   esmacat_err  set_robot_referenced(bool robot_referenced);
   bool         is_robot_referenced();

   void         set_conversion_factor(int conv){conversion_factor = conv;}
   int       get_conversion_factor();

   esmacat_err set_calibration_reference_counts(int index, int32_t input_calibration_reference_counts);
   int32_t     get_calibration_reference_counts(int index);

   esmacat_err set_calibration_ready(bool calibration_ready);
   bool        is_calibration_ready();

   int16_t     get_input_variable(int index);
   esmacat_err set_output_variable(int index, int16_t output_variable);

};

#endif // AGREE_EASE_H
