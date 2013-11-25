from django.http import HttpResponse
from django.shortcuts import render_to_response
from django.views.decorators.csrf import csrf_exempt
from django.template import RequestContext

history = []

@csrf_exempt
def message(request):
	if request.method == 'POST':
		msg_text = request.body #request.POST.get("message", "EMPTY MESSAGE")
		history.append(msg_text)
		return HttpResponse()
	else:
		return render_to_response('history.html', {'history': history},
                              context_instance=RequestContext(request))
