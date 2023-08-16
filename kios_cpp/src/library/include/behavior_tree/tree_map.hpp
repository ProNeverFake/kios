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
    static const char *test_tree = R"(
        <root BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <Fallback name="root_sequence">
                    <Contact name="contact"/>
                </Fallback>
            </BehaviorTree>
        </root>
        )";
} // namespace Insertion