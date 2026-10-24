package org.firstinspires.ftc.teamcode;

import java.util.Set;

public class Judge {
    /*这里使用了一个新的集合类型，即Set，可以理解成一个不允许重复元素的列表，其中Integer表示集合中的数要是整数
    * 其好处是可以通过contains()来快速判断某个值在不在集合中，不用写一长串||，也方便修改，这里也是出于人道主义精神使用更为美观的Set*/
    private static final Set<Integer> RED_LEFT = Set.of(30 , 31 , 32 , 33);
    private static final Set<Integer> RED_RIGHT = Set.of(34 , 35 , 36 , 37);
    private static final Set<Integer> BLUE_LEFT = Set.of(42 , 43 , 44 , 45);
    private static final Set<Integer> BLUE_RIGHT = Set.of(38 , 39 , 40 , 41);
    private final Limelight limelight;
    public Judge(Limelight limelight){
        this.limelight = limelight;
    }

    /*四个返回值为布尔值的方法，负责判断对应的蜂巢是否为朝上的那个，从而在自动阶段判断是否shoot*/
    public boolean isRedLeft(){
        return limelight.ishasTarget() && RED_LEFT.contains(limelight.getTagId());
    }

    public boolean isRedRight(){
        return limelight.ishasTarget() && RED_RIGHT.contains(limelight.getTagId());
    }

    public boolean isBlueLeft(){
        return limelight.ishasTarget() && BLUE_LEFT.contains(limelight.getTagId());
    }

    public boolean isBlueRight(){
        return limelight.ishasTarget() && BLUE_RIGHT.contains(limelight.getTagId());
    }
}
