function Cqa_one = calculate_Cqa_one(Ai,Aj,upi_,upj_,vi1_,vi2_,vj_)
Id  = eye(3,3);
Cqa_one = [Id,-Ai*skew(upi_),-Id,Aj*skew(upj_);
            0,0,0,-(Aj*vj_)'*Ai*skew(vi1_),0,0,0,-(Ai*vi1_)'*Aj*skew(vj_);
            0,0,0,-(Aj*vj_)'*Ai*skew(vi2_),0,0,0,-(Ai*vi2_)'*Aj*skew(vj_)];
end