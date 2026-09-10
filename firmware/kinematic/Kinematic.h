typedef enum {
    x,
    y,
    z,
} Kin_Coor_TypeDef;

typedef struct {
    Joint_Stepper_Typedef hj[10];

    Kin_Coor_TypeDef coor;

    float joint_len;
} Kin_TypeDef;


