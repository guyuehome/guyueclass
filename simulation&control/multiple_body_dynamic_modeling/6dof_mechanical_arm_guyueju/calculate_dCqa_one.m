function dCqa_one = calculate_dCqa_one(Ai,dAi,Aj,dAj,upi_,upj_,vi1_,vi2_,vj_)
I0  = [0,0,0;0,0,0;0,0,0];
dCqa_one = [I0,-dAi*skew(upi_),I0,dAj*skew(upj_);
            0,0,0,-(dAj*vj_)'*Ai*skew(vi1_)-(Aj*vj_)'*dAi*skew(vi1_),0,0,0,-(dAi*vi1_)'*Aj*skew(vj_)-(Ai*vi1_)'*dAj*skew(vj_);
            0,0,0,-(dAj*vj_)'*Ai*skew(vi2_)-(Aj*vj_)'*dAi*skew(vi2_),0,0,0,-(dAi*vi2_)'*Aj*skew(vj_)-(Ai*vi2_)'*dAj*skew(vj_)];
end