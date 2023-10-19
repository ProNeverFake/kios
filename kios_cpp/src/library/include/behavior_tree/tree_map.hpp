namespace Insertion
{
    // static const char *tree = R"(
    //     <root BTCPP_format="4" >

    //         <BehaviorTree ID="MainTree">
    //             <ReactiveSequence name="root_sequence">

    //                 <CheckFinished   name="check_finished"/>

    //                 <SubTree ID="push_peg_z"/>

    //                 <ReaciveSequence name="peg_align_hole">
    //                     <CheckAlign  name="check_align"/>
    //                     <Align name="align">
    //                 </ReactiveSequence>

    //                 <ReaciveSequence name="peg_fit_hole">
    //                     <CheckFit  name="check_fit"/>
    //                     <Fit name="fit_hole">
    //                 </ReactiveSequence>

    //                 <ReaciveSequence name="peg_reach_hole">
    //                     <CheckReach  name="check_reach"/>
    //                     <ReachHole name="reach_hole">
    //                 </ReactiveSequence>

    //                 <Approach name="approach"/>
    //             </ReacitveSequence>
    //         </BehaviorTree>

    //         <BehaviorTree ID="push_peg_z">
    //             <ReaciveSequence name="push_peg">
    //                 <CheckPush  name="check_push"/>
    //                 <Push name="push_peg">
    //             </ReactiveSequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <Approach name="approach"/>
    //                 <Contact name="contact"/>
    //                 <Wiggle name="wiggle"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <Approach name="approach"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here cartesian move test
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <CartesianMove name="cartesian_move" action_id="1" description="test" objects="housing"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here gripper move test
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <GripperMove name="gripper_move" action_id="2" description="test"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here joint move test
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <JointMove name="joint_move" action_id="3" description="test" objects="housing"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here gripper force test
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <GripperForce name="gripper_force" action_id="4" description="test"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here a mini task
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <JointMove name="joint_move" action_id="1" description="test" objects="initial"/>
    //                 <JointMove name="joint_move" action_id="2" description="test" objects="obj1"/>
    //                 <GripperForce name="gripper_force" action_id="3" description="test"/>
    //                 <CartesianMove name="cart_move" action_id="4" description="test" objects="obj2"/>
    //                 <JointMove name="joint_move" action_id="5" description="test" objects="obj3"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here a tool load task
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <ToolLoad name="joint_move" action_id="20" description="test" objects="tool1"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here a tool unload task
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <ToolUnload name="tool_unload" action_id="21" description="test" objects="tool1"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here a tool load + unload task
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <ToolLoad name="joint_move" action_id="20" description="test" objects="tool1"/>
    //                 <JointMove name="joint_move" action_id="22" description="test" objects="joint1"/>
    //                 <ToolUnload name="tool_unload" action_id="21" description="test" objects="tool1"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here tool grasp test
    static const char *test_tree = R"(
        <root BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <Sequence name="root_sequence">
                    <ToolGrasp name="tool_grasp" action_id="23" description="test" objects="tool_grasp"/>
                </Sequence>
            </BehaviorTree>
        </root>
        )";

    // ! here contact test
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <Contact name="contact" action_id="10" description="test" objects="obj3"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // ! here wiggle test
    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <Wiggle name="wiggle" action_id="11" description="test" objects="obj3"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <CartesianMove name="cartesian_move" action_id="1" description="cartesian_move"/>
    //                 <CartesianMove name="cartesian_move2" action_id="2" objects="something"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";

    // static const char *test_tree = R"(
    //     <root BTCPP_format="4" >
    //         <BehaviorTree ID="MainTree">
    //             <Sequence name="root_sequence">
    //                 <Fallback name="approach_fallback">
    //                     <AtPositionApproch name="at_position_approach"/>
    //                     <Approach name="approach"/>
    //                 </Fallback>
    //                 <Sequence name="approach_sequence">
    //                     <HasObjectContact name="has_object_contact"/>
    //                     <Contact name="contact"/>
    //                 </Sequence>
    //                 <Wiggle name="wiggle"/>
    //             </Sequence>
    //         </BehaviorTree>
    //     </root>
    //     )";
} // namespace Insertion