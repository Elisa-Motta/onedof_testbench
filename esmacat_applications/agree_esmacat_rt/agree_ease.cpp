#include "agree_ease.h"

agree_ease::agree_ease()
{

}

esmacat_err agree_ease::set_robot_referenced(bool robot_referenced){
    esmacat_err err = NO_ERR;
    err = set_output_variable(6,1);
    return err;
}

bool agree_ease::is_robot_referenced(){
    // get
    if(get_input_variable(6)==1)
        robot_referenced = true;
    else
        robot_referenced = false;
    return robot_referenced;
}

esmacat_err agree_ease::set_calibration_ready(bool calibration_ready){
    esmacat_err err = NO_ERR;
    err = set_output_variable(7,1);
    return err;
}

bool agree_ease::is_calibration_ready(){

    if(get_input_variable(7)==1)
        calibration_ready = true;
    else
        calibration_ready = false;

    return calibration_ready;
}

esmacat_err agree_ease::set_calibration_reference_counts(int index, int32_t output_calibration_reference_counts){
    esmacat_err err = NO_ERR;
    err = set_output_variable(index,static_cast<int16_t>(output_calibration_reference_counts/conversion_factor));
    return err;
}

int32_t agree_ease::get_calibration_reference_counts(int index){
    calibration_reference_counts[index] = static_cast<int32_t>(get_input_variable(index)*conversion_factor);
    return calibration_reference_counts[index];
}

esmacat_err agree_ease::set_output_variable(int index, int16_t output_variable){
    // Initialize error to NO_ERR
    esmacat_err err = NO_ERR;
    // Switch variable index
    switch (index){
    case 0:
        set_output_variable_0_OUT_GEN_INT0(output_variable);
        break;
    case 1:
        set_output_variable_1_OUT_GEN_INT1(output_variable);
        break;
    case 2:
        set_output_variable_2_OUT_GEN_INT2(output_variable);
        break;
    case 3:
        set_output_variable_3_OUT_GEN_INT3(output_variable);
        break;
    case 4:
        set_output_variable_4_OUT_GEN_INT4(output_variable);
        break;
    case 5:
        set_output_variable_5_OUT_GEN_INT5(output_variable);
        break;
    case 6:
        set_output_variable_6_OUT_GEN_INT6(output_variable);
        break;
    case 7:
        set_output_variable_7_OUT_GEN_INT7(output_variable);
        break;

    default:
        // If index is out of range, raise error
        err =  ERR_UNKNOWN;
        break;
    }
    return err;
}

int16_t agree_ease::get_input_variable(int index){
    // Initialize error to NO_ERR
    esmacat_err err = NO_ERR;
    // Initialize input variable
    int16_t input_variable = 0;

    // Switch variable index
    switch (index){
    case 0:
        input_variable = get_input_variable_0_IN_GEN_INT0();
        break;
    case 1:
        input_variable = get_input_variable_1_IN_GEN_INT1();
        break;
    case 2:
        input_variable = get_input_variable_2_IN_GEN_INT2();
        break;
    case 3:
        input_variable = get_input_variable_3_IN_GEN_INT3();
        break;
    case 4:
        input_variable = get_input_variable_4_IN_GEN_INT4();
        break;
    case 5:
        input_variable = get_input_variable_5_IN_GEN_INT5();
        break;
    case 6:
        input_variable = get_input_variable_6_IN_GEN_INT6();
        break;
    case 7:
        input_variable = get_input_variable_7_IN_GEN_INT7();
        break;

    default:
        // If index is out of range, raise error
        err =  ERR_UNKNOWN;
        break;
    }
    return input_variable;
}
