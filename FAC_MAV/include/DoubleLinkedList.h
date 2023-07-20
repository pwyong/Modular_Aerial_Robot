#ifndef DOUBLELINKEDLIST_H
#define DOUBLELINKEDLIST_H
#include <iostream>
#include <string>

// 2023.07.12
// This DoubleLinkedList class is exclusively for MHE (Moving Horizon Estimation) implementation
// You can edit freely for use in other purposes

//class DoubleLinkedList;
class Node{
	friend class DoubleLinkedList;
	private:
		Node *next; // Next node pointer
		Node *prev; // Previous node pointer
	public:
		double data;
		Node(Node *n, Node *p, double v){
			next = n;
			prev = p;
			data = v;
		}	
		Node(double v){next = prev = NULL; data=v;}
};

class DoubleLinkedList{
	private:
		Node *head=NULL;
		Node *tail=NULL;
		int size=0;
	public:
		DoubleLinkedList(){head=tail=NULL; size=0;}
		~DoubleLinkedList(){
			Node *cur = head;
			while (cur != NULL){
				deleteHead();
			}
		}
		void insertHead(double value){
			Node *newNode=new Node(value);
			if(head==NULL && tail==NULL){
				head=newNode;
				tail=head;
			}
			else{
				newNode->next=head;
				head->prev=newNode;
			}
			head=newNode;
			size++;
		}
		void insertTail(double value){
			Node *newNode=new Node(value);
			if(tail==NULL && head==NULL){
				tail=newNode;
				head=tail;
			}
			else{
				newNode->prev=tail;
				tail->next=newNode;
			}
			tail=newNode;
			size++;
		}
		void deleteHead(){
			if(head!=NULL){
				Node *remove=head;
				head=head->next;
				head->prev=NULL;
				delete remove;
				size--;
			}
		}
		void deleteTail(){
			if(tail!=NULL){
				Node *remove=tail;
				tail=tail->prev;
				tail->next=NULL;
				delete remove;
				size--;
			}
		}
		double getData(int index){
			Node *curNode=head;
			if(index==0) {
				if(!std::isnan(head->data)) return head->data;
				else return 0;
			}	
			else if(index < size){
				for(int i=0;i<index;i++){
					curNode=curNode->next;
				}
				if(!std::isnan(curNode->data))	return curNode->data;
				else return 0;
			}
			else return 0;

		}
		int length(){
			return size;
		}
	//explicit DoubleLinkedList(double value);
};
#endif
