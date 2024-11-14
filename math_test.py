from Tkinter import Tk, Button, Label, Entry, StringVar
import sys
import signal
from datetime import datetime
import random
import math_test_set as math_set
import time

def signal_handler(sig, frame):
	print "\nCleaning up..."
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

MATH = {"EASY" : math_set.get_easy(), "MEDIUM" : math_set.get_medium(), "HARD" : math_set.get_hard()}
DIFFICULTIES = ["EASY", "MEDIUM", "HARD"]

problem_times = {}

TASK_START_TIME = 0

class Test():
	def get_time(self):
		return datetime.now().minute*60+datetime.now().second - TASK_START_TIME
	def set_textbox(self):
		print "Setting text box"
		self.math_text.set(self.current_math_question)
		self.entry_box.delete(0,'end')

	def submit_button_handler(self, event=None):
		if not self.started:
			self.started = True
			self.start_time = self.get_time()
			self.math_text.set(self.current_math_question)
			return
		try:
			answer = int(self.entry_box.get().translate(None, " ")) #Takes the input from the entry box, strips away any spaces, and converts it into an integer
			self.entry_box.delete(0,'end') #Clear the entry box
		except ValueError:
			self.math_text.set("Answer must be a whole number")
			print "Set math text"
			self.submit_button.after(3000, self.set_textbox)
			self.start_time = self.get_time()
			return
		if self.difficulty_index > 2:
			self.math_text.set("You're all done!")
			raw_input("Press enter...")
			print "\n-----------------------------------\n"
			print "Difficulty: HARD"
			signal_handler(signal.SIGINT, None)
			
		elif(answer == MATH[DIFFICULTIES[self.difficulty_index]][self.current_math_question]):
			self.questions_correct += 1
		self.questions_done += 1
		self.end_time = self.get_time()
		self.times.append(self.end_time-self.start_time)
		if self.questions_done == 5:
			self.avg_time = sum(self.times)/float(len(self.times))
			print self.avg_time
		if self.questions_done == 10:
			self.avg_time = sum(self.times)/float(len(self.times))
			print self.avg_time
		if(self.questions_done > 5 and self.avg_time <= 8 and self.questions_correct/self.questions_done >= 0.8): #<8 second time and 80% correct
			self.next_set()
		elif self.questions_done == 10:
			self.math_questions = None
		if not self.math_questions:
			self.math_text.set("You're all done!")
			print "\n-----------------------------------\n"
			print "Difficulty:"
			print DIFFICULTIES[self.difficulty_index]
			time.sleep(3)
			signal_handler(signal.SIGINT, None)
		self.current_math_question = self.math_questions.pop(0)
		self.math_text.set(self.current_math_question)
		self.start_time = self.get_time()

	def next_set(self):
		self.difficulty_index += 1
		if self.difficulty_index > 2:
			if(self.questions_done > 5 and self.avg_time <= 8 and self.questions_correct/self.questions_done >= 0.8):
				raw_input("Press enter...")
				print "\n-----------------------------------\n"
				print "Difficulty:"
				try:
					print DIFFICULTIES[self.difficulty_index-1]
				except IndexError:
					print "HARD"
			else:
				raw_input("Pres enter...")
				print "\n-----------------------------------\n"
				print "Difficulty:"
				try:
					print DIFFICULTIES[self.difficulty_index-2]
				except IndexError:
					print "MEDIUM"
			signal_handler(signal.SIGINT, None)
		print "Difficulty:", DIFFICULTIES[self.difficulty_index]
		self.math_questions = MATH[DIFFICULTIES[self.difficulty_index]].keys()
		self.current_math_question = self.math_questions.pop(0)
		self.avg_time = 100
		self.questions_done = 0
		self.questions_correct = 0
		self.times = []


	def __init__(self):
		self.root = Tk()
		self.root.bind('<KP_Enter>', self.submit_button_handler)
		self.root.bind('<Return>', self.submit_button_handler)
		self.root.minsize(640,360)
		self.root.grid_rowconfigure(0, weight=1)
		self.root.grid_rowconfigure(4, weight=1)
		self.root.grid_columnconfigure(0, weight=1)
		self.root.grid_columnconfigure(2, weight=1)
		self.math_problems = {}

		self.started=False

		self.submit_button = Button(self.root, command=self.submit_button_handler, text="Enter", font=('Arial', 16), height=2, width=8, bg='green')
		#self.submit_button.place(x=260, y=280)


		self.entry_box = Entry(self.root, font=('Arial', 18))
		#self.entry_box.place(x=190, y=220)


		self.math_text = StringVar()
		self.math_text.set("Press enter to begin")

		self.math_label = Label(self.root, textvariable=self.math_text, font=('Arial', 18), height=2, width=28, bg='white')
		#self.math_label.place(x=130, y=150)
		self.math_label.grid(row=1,column=1, pady=5)
		self.entry_box.grid(row=2,column=1, pady=5)
		self.submit_button.grid(row=3,column=1, pady=5)
		self.math_questions = MATH["EASY"].keys()
		self.current_math_question = self.math_questions.pop(0)
		self.difficulty_index = 0
		self.questions_done = 0
		self.questions_correct = 0
		self.avg_time = 100
		self.times = []

	def start(self):
		global TASK_START_TIME
		TASK_START_TIME = self.get_time()
		self.root.mainloop()

if __name__ == "__main__":
	t = Test()
	t.start()
